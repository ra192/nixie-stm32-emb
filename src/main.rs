//! 6-digit Nixie tube clock for STM32F103 ("Blue Pill") using Embassy.
//!
//! Dynamic (multiplexed) indication:
//!   - the 10 digit cathodes (0-9) of all tubes are wired together,
//!   - only ONE tube's anode (grid) is powered at a time,
//!   - at any instant exactly one cathode + one grid are driven,
//!   - scanning all 6 tubes ~ every 6 ms makes them appear constantly lit.
//!
//! WIRING (custom discrete circuit, drives the high-voltage stage 3.3V side):
//!
//!   Digit cathodes  (shared, ACTIVE LOW, low-side switch e.g. NPN -> K155ID1/74141
//!   or MPSA42+220k to the cathode on the 170V rail):
//!     PA3 -> digit 0      PA12 -> digit 5
//!     PA5 -> digit 1      PB3 -> digit 6
//!     PA7 -> digit 2      PB11 -> digit 7
//!     PB0 -> digit 3      PB10 -> digit 8
//!     PB4 -> digit 4      PB2 -> digit 9
//!
//!   Anode / grid select (ACTIVE HIGH, high-side PNP/MPSA92 switch, one per tube):
//!     PB5 -> tube 1  (hours  tens)
//!     PB1 -> tube 2  (hours  ones)
//!     PB9 -> tube 3  (minutes tens)
//!     PA4 -> tube 4  (minutes ones)
//!     PA6 -> tube 5  (seconds tens)
//!     PA8 -> tube 6  (seconds ones)
//!
//!   Controls (buttons to GND, internal pull-up => active LOW):
//!     PB12 -> MODE  (cycles edit field: hours -> minutes -> seconds -> run)
//!     PB13 -> ADJUST (increments the edited field, auto-repeat while held)
//!
//!   Status:
//!     PC13 -> on-board Blue Pill LED (negative logic: LOW = lit)
//!
//!   Real-time clock (DS3231 at I2C address 0x68, read once per minute):
//!     PB6 -> SCL      PB7 -> SDA
//!
//! Clock source for the core: 8 MHz HSI /2 x16 = 64 MHz.

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Blocking;
use embassy_stm32::Config;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::rcc::{AHBPrescaler, APBPrescaler, Pll, PllMul, PllPreDiv, PllSource, Sysclk};
use embassy_time::{Duration, Instant, Timer};
use panic_probe as _;

const NUM_TUBES: usize = 6;
/// Value used to blank a tube (no cathode driven).
const BLANK: u8 = 10;
/// Time each tube is lit per scan step.
const SLICE_MS: u64 = 1;
/// Key debounce / hold-before-auto-repeat.
const DEBOUNCE: Duration = Duration::from_millis(30);
const REPEAT_INITIAL: Duration = Duration::from_millis(400);
const REPEAT_STEP: Duration = Duration::from_millis(120);
/// Default time shown at power-on (hour, minute, second).
const START_HOUR: u8 = 12;
const START_MIN: u8 = 0;
const START_SEC: u8 = 0;

// ---------------------------------------------------------------------------
// Nixie display
// ---------------------------------------------------------------------------
struct Display {
    // Segment cathodes, active LOW (Low = digit on).
    segments: [Output<'static>; 10],
    // Anode grids, active HIGH (High = tube powered).
    grids: [Output<'static>; NUM_TUBES],
    digits: [u8; NUM_TUBES],
    scan: usize,
}

impl Display {
    fn new(segments: [Output<'static>; 10], grids: [Output<'static>; NUM_TUBES]) -> Self {
        Self {
            segments,
            grids,
            digits: [BLANK; NUM_TUBES],
            scan: 0,
        }
    }

    fn set_digits(&mut self, digits: [u8; NUM_TUBES]) {
        self.digits = digits;
    }

    /// Switch everything off (prevents ghosting between tubes).
    fn all_off(&mut self) {
        for s in &mut self.segments {
            s.set_low();
        }
        for g in &mut self.grids {
            g.set_low();
        }
    }

    /// Drive one tube for one scan step, then advance to the next tube.
    fn step(&mut self) {
        self.all_off();

        let d = self.digits[self.scan];
        if d < 10 {
            self.segments[d as usize].set_high(); // enable that cathode
        }
        self.grids[self.scan].set_high(); // power that tube's anode

        self.scan = (self.scan + 1) % NUM_TUBES;
    }
}

// ---------------------------------------------------------------------------
// Software clock (HH:MM:SS)
// ---------------------------------------------------------------------------
struct Clock {
    hour: u8,
    min: u8,
    sec: u8,
}

impl Clock {
    fn new(hour: u8, min: u8, sec: u8) -> Self {
        Self {
            hour: hour % 24,
            min: min % 60,
            sec: sec % 60,
        }
    }

    fn tick(&mut self) {
        self.sec += 1;
        if self.sec >= 60 {
            self.sec = 0;
            self.min += 1;
            if self.min >= 60 {
                self.min = 0;
                self.hour = (self.hour + 1) % 24;
            }
        }
    }

    /// [Hh, Hl, Mh, Ml, Sh, Sl], BLANK allowed for the hours-tens "0".
    fn digits(&self) -> [u8; NUM_TUBES] {
        [
            if self.hour < 10 {
                BLANK
            } else {
                self.hour / 10
            },
            self.hour % 10,
            self.min / 10,
            self.min % 10,
            self.sec / 10,
            self.sec % 10,
        ]
    }
}

// ---------------------------------------------------------------------------
// DS3231 real-time clock (I2C)
// ---------------------------------------------------------------------------
/// DS3231 7-bit I2C address (A0/A1/AD0 tied low).
const RTC_ADDR: u8 = 0x68;

/// Decode a binary-coded-decimal byte (0x00-0x99) into a plain value.
fn bcd(b: u8) -> u8 {
    (b & 0x0F) + 10 * (b >> 4)
}

struct Ds3231 {
    i2c: I2c<'static, Blocking, Master>,
}

impl Ds3231 {
    fn new(i2c: I2c<'static, Blocking, Master>) -> Self {
        Self { i2c }
    }

    /// Read HH:MM:SS from the RTC and copy it into [Clock] if it looks sane.
    fn sync(&mut self, clock: &mut Clock) {
        // Point the address register at seconds (0x00) and read 3 registers.
        let mut buf = [0u8; 3];
        match self.i2c.blocking_write_read(RTC_ADDR, &[0x00], &mut buf) {
            Err(e) => info!("rtc: read error {:?}", e),
            Ok(()) => {
                let sec = bcd(buf[0] & 0x7F); // bit7 = clock-halt flag
                let min = bcd(buf[1]);
                let hour = bcd(buf[2] & 0x3F); // bit6 = 0 in 24-hour mode
                if hour < 24 && min < 60 && sec < 60 {
                    clock.hour = hour;
                    clock.min = min;
                    clock.sec = sec;
                    info!("rtc: {:02}:{:02}:{:02}", hour, min, sec);
                } else {
                    info!("rtc: invalid time {:02}:{:02}:{:02}", hour, min, sec);
                }
            }
        }
    }

    /// Write HH:MM:SS to the RTC in BCD (24-hour mode, clock halt cleared).
    fn save(&mut self, clock: &Clock) {
        let buf = [
            0x00,                                        // register pointer (seconds)
            ((clock.sec / 10) << 4) | (clock.sec % 10), // bit7 (CH) = 0: run
            ((clock.min / 10) << 4) | (clock.min % 10),
            ((clock.hour / 10) << 4) | (clock.hour % 10), // bit6 = 0: 24h mode
        ];
        match self.i2c.blocking_write(RTC_ADDR, &buf) {
            Ok(()) => info!("rtc: stored {:02}:{:02}:{:02}", clock.hour, clock.min, clock.sec),
            Err(e) => info!("rtc: write error {:?}", e),
        }
    }
}

// ---------------------------------------------------------------------------
// Debounced button (active LOW)
// ---------------------------------------------------------------------------
struct DebouncedButton {
    pin: Input<'static>,
    raw: bool,
    stable: bool,
    was_pressed: bool,
    since: Instant,
}

impl DebouncedButton {
    fn new(pin: Input<'static>) -> Self {
        let raw = pin.is_high();
        Self {
            pin,
            raw,
            stable: raw,
            was_pressed: false,
            since: Instant::now(),
        }
    }

    /// Update debounced state, return `true` while the button is stably pressed.
    fn pressed(&mut self) -> bool {
        let now = Instant::now();
        let r = self.pin.is_high();
        if r != self.raw {
            self.raw = r;
            self.since = now;
        } else if now >= self.since + DEBOUNCE {
            self.stable = r;
        }
        !self.stable
    }

    /// Return `true` once on each stable press (falling-edge detection).
    fn just_pressed(&mut self) -> bool {
        if self.pressed() {
            if !self.was_pressed {
                self.was_pressed = true;
                return true;
            }
        } else {
            self.was_pressed = false;
        }
        false
    }
}

// ---------------------------------------------------------------------------
// Edit state machine
// ---------------------------------------------------------------------------
#[derive(Clone, Copy, PartialEq)]
enum EditField {
    None,
    Hours,
    Minutes,
    Seconds,
}

impl EditField {
    fn next(self) -> Self {
        match self {
            Self::None => Self::Hours,
            Self::Hours => Self::Minutes,
            Self::Minutes => Self::Seconds,
            Self::Seconds => Self::None,
        }
    }

    /// Indices of the two tubes that make up this 2-digit field.
    fn tubes(self) -> (usize, usize) {
        match self {
            Self::Hours => (0, 1),
            Self::Minutes => (2, 3),
            Self::Seconds => (4, 5),
            Self::None => (0, 0),
        }
    }
}

fn edit_name(f: EditField) -> &'static str {
    match f {
        EditField::None => "run",
        EditField::Hours => "hours",
        EditField::Minutes => "minutes",
        EditField::Seconds => "seconds",
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    // 8 MHz HSI -> PLL /2 x16 -> 64 MHz.
    config.rcc.hsi = true;
    config.rcc.pll = Some(Pll {
        src: PllSource::HSI,
        // Note: when the PLL source is HSI on STM32F1, prediv must be 2.
        prediv: PllPreDiv::DIV2,
        mul: PllMul::MUL16,
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV2;
    config.rcc.apb2_pre = APBPrescaler::DIV1;
    let p = embassy_stm32::init(config);

    info!("nixie clock: STM32F103C8 startup");

    // Digit cathodes. Idle = High (off), digit selected = Low.
    let segments = [
        Output::new(p.PA3, Level::High, Speed::Low),
        Output::new(p.PA5, Level::High, Speed::Low),
        Output::new(p.PA7, Level::High, Speed::Low),
        Output::new(p.PB0, Level::High, Speed::Low),
        Output::new(p.PB4, Level::High, Speed::Low),
        Output::new(p.PA12, Level::High, Speed::Low),
        Output::new(p.PB3, Level::High, Speed::Low),
        Output::new(p.PB11, Level::High, Speed::Low),
        Output::new(p.PB10, Level::High, Speed::Low),
        Output::new(p.PB2, Level::High, Speed::Low),
    ];

    // Anode grids. Idle = Low (off), tube powered = High.
    let grids = [
        Output::new(p.PB5, Level::Low, Speed::Low),
        Output::new(p.PB1, Level::Low, Speed::Low),
        Output::new(p.PB9, Level::Low, Speed::Low),
        Output::new(p.PA4, Level::Low, Speed::Low),
        Output::new(p.PA6, Level::Low, Speed::Low),
        Output::new(p.PA8, Level::Low, Speed::Low),
    ];

    let mut display = Display::new(segments, grids);

    // Buttons (active LOW).
    let mut btn_mode = DebouncedButton::new(Input::new(p.PA1, Pull::Up));
    let mut btn_minus = DebouncedButton::new(Input::new(p.PA2, Pull::Up));
    let mut btn_plus = DebouncedButton::new(Input::new(p.PA0, Pull::Up));

    // DS3231 on I2C1 (SCL=PB6, SDA=PB7). 100 kHz default.
    let mut rtc = Ds3231::new(I2c::new_blocking(
        p.I2C1,
        p.PB6,
        p.PB7,
        embassy_stm32::i2c::Config::default(),
    ));

    let mut clock = Clock::new(START_HOUR, START_MIN, START_SEC);
    rtc.sync(&mut clock); // take the boot time from the RTC
    let mut field = EditField::None;

    let mut next_sec = Instant::now() + Duration::from_secs(1);
    let mut next_adj = Instant::now() + REPEAT_INITIAL;
    let mut next_rtc = Instant::now() + Duration::from_secs(60);

    loop {
        let now = Instant::now();

        // --- one multiplex slice per iteration -----------------------------
        display.step();

        // --- edit mode -----------------------------------------------------
        if btn_mode.just_pressed() {
            if field == EditField::Seconds {
                rtc.save(&clock);
            }
            field = field.next();
            info!("edit: {}", edit_name(field));
        }

        if field != EditField::None && btn_plus.pressed() {
            if now >= next_adj {
                match field {
                    EditField::Hours => clock.hour = (clock.hour + 1) % 24,
                    EditField::Minutes => clock.min = (clock.min + 1) % 60,
                    EditField::Seconds => clock.sec = (clock.sec + 1) % 60,
                    EditField::None => {}
                }
                info!("time: {:02}:{:02}:{:02}", clock.hour, clock.min, clock.sec);
                next_adj = now + REPEAT_STEP;
            }
        } else if field != EditField::None && btn_minus.pressed() {
            if now >= next_adj {
                match field {
                    EditField::Hours => clock.hour = (clock.hour + 23) % 24,
                    EditField::Minutes => clock.min = (clock.min + 59) % 60,
                    EditField::Seconds => clock.sec = (clock.sec + 59) % 60,
                    EditField::None => {}
                }
                info!("time: {:02}:{:02}:{:02}", clock.hour, clock.min, clock.sec);
                next_adj = now + REPEAT_STEP;
            }
        } else {
            // Reload the longer initial delay each time the button is released.
            next_adj = now + REPEAT_INITIAL;
        }

        // --- timekeeping (only in run mode) --------------------------------
        while field == EditField::None && now >= next_sec {
            clock.tick();
            next_sec += Duration::from_secs(1);
        }

        // --- re-sync from the RTC once per minute (skip while editing) -----
        if field == EditField::None && now >= next_rtc {
            rtc.sync(&mut clock);
            next_rtc += Duration::from_secs(60);
        }

        // --- display content ------------------------------------------------
        let mut d = clock.digits();
        if field != EditField::None && (now.as_millis() / 500).is_multiple_of(2) {
            // Blink the edited 2-digit field at 1 Hz.
            let (a, b) = field.tubes();
            d[a] = BLANK;
            d[b] = BLANK;
        }
        display.set_digits(d);

        Timer::after_millis(SLICE_MS).await;
    }
}
