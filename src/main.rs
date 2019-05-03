#![no_std]
#![no_main]

// Busy A4
// CS   A3
// SCK  A5
// MOSI A7
// DC   A2
// RST  A1
//
// pick a panicking behavior
// extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

use rtfm::app;
use rtfm::export::wfi;

use embedded_graphics::prelude::*;
use embedded_graphics::{drawable::Pixel, prelude::UnsignedCoord, Drawing};
use profont::{ProFont12Point, ProFont14Point, ProFont24Point, ProFont9Point};

use embedded_hal::spi::MODE_0;
use ssd1675::{
    command::{BufCommand, Command, DataEntryMode, IncrementAxis},
    interface::{DisplayInterface, Interface},
};
use stm32f1::stm32f103::Interrupt;
use stm32f1xx_hal::{
    delay::Delay,
    gpio::{
        gpioa::{PA1, PA2, PA3, PA4},
        gpiob::{PB5, PB6, PB7, PB8},
        Output, PushPull,
    },
    prelude::*,
    serial::{Rx, Serial, Tx},
    spi::Spi,
    i2c::{I2c, Mode, blocking_i2c},
    timer::{Event, Timer},
};
use htu21d::Htu21df;
use bmpx85::Bmpx85;

// Debug
use cortex_m_semihosting::hio;
use core::fmt::Write;

const LUT: [u8; 70] = [
    0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00, //LUT0: BB:     VS 0 ~7
    0x10, 0x60, 0x20, 0x00, 0x00, 0x00, 0x00, //LUT1: BW:     VS 0 ~7
    0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00, //LUT2: WB:     VS 0 ~7
    0x10, 0x60, 0x20, 0x00, 0x00, 0x00, 0x00, //LUT3: WW:     VS 0 ~7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //LUT4: VCOM:   VS 0 ~7
    0x03, 0x03, 0x00, 0x00, 0x02, // TP0 A~D RP0
    0x09, 0x09, 0x00, 0x00, 0x02, // TP1 A~D RP1
    0x03, 0x03, 0x00, 0x00, 0x02, // TP2 A~D RP2
    0x00, 0x00, 0x00, 0x00, 0x00, // TP3 A~D RP3
    0x00, 0x00, 0x00, 0x00, 0x00, // TP4 A~D RP4
    0x00, 0x00, 0x00, 0x00, 0x00, // TP5 A~D RP5
    0x00, 0x00, 0x00, 0x00, 0x00, // TP6 A~D RP6
];

// Max display resolution is 250x128
/// The maximum number of rows supported by the controller
pub const MAX_GATE_OUTPUTS: u16 = 250;
/// The maximum number of columns supported by the controller
pub const MAX_SOURCE_OUTPUTS: u8 = 128;

pub const ROWS: u16 = 128;
pub const COLS: u16 = 250;

// Magic numbers from the data sheet
const ANALOG_BLOCK_CONTROL_MAGIC: u8 = 0x54;
const DIGITAL_BLOCK_CONTROL_MAGIC: u8 = 0x3B;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Color {
    Black,
    White,
}

impl PixelColor for Color {}

impl From<u8> for Color {
    fn from(value: u8) -> Self {
        match value {
            0 => Color::Black,
            1 => Color::White,
            _ => panic!("invalid color value"),
        }
    }
}

struct Display<'a, SPI, CS, BUSY, DC, RESET>
where
    SPI: embedded_hal::blocking::spi::Write<u8>,
    CS: embedded_hal::digital::OutputPin,
    BUSY: embedded_hal::digital::InputPin,
    DC: embedded_hal::digital::OutputPin,
    RESET: embedded_hal::digital::OutputPin,
{
    interface: Interface<SPI, CS, BUSY, DC, RESET>,
    buff: &'a mut [u8],
}

impl<'a, SPI, CS, BUSY, DC, RESET> Display<'a, SPI, CS, BUSY, DC, RESET>
where
    SPI: embedded_hal::blocking::spi::Write<u8>,
    CS: embedded_hal::digital::OutputPin,
    BUSY: embedded_hal::digital::InputPin,
    DC: embedded_hal::digital::OutputPin,
    RESET: embedded_hal::digital::OutputPin,
{
    fn init<D: embedded_hal::blocking::delay::DelayMs<u8>>(&mut self, delay: &mut D) {
        self.interface.reset(delay);
        Command::SoftReset.execute(&mut self.interface);
        self.interface.busy_wait();
        Command::AnalogBlockControl(ANALOG_BLOCK_CONTROL_MAGIC).execute(&mut self.interface);
        Command::DigitalBlockControl(DIGITAL_BLOCK_CONTROL_MAGIC).execute(&mut self.interface);
        Command::DriverOutputControl(0xF9, 0x00).execute(&mut self.interface);
        Command::DataEntryMode(
            DataEntryMode::IncrementYIncrementX,
            IncrementAxis::Horizontal,
        )
        .execute(&mut self.interface);
        Command::StartEndXPosition(0x0, 0x0F).execute(&mut self.interface); //0x0F-->(15+1)*8=128
        Command::StartEndYPosition(0x0, 0xF9).execute(&mut self.interface); //0xF9-->(249+1)=250
        Command::BorderWaveform(0x03).execute(&mut self.interface);
        Command::WriteVCOM(0x55).execute(&mut self.interface);
        Command::GateDrivingVoltage(0x15).execute(&mut self.interface);
        Command::SourceDrivingVoltage(0x41, 0xA8, 0x32).execute(&mut self.interface);
        Command::DummyLinePeriod(0x30).execute(&mut self.interface);
        Command::GateLineWidth(0x0A).execute(&mut self.interface);
        BufCommand::WriteLUT(&LUT).execute(&mut self.interface);
        self.interface.busy_wait();
    }

    fn set_pixel(&mut self, x: u32, y: u32, color: Color) {
        let (index, bit) = get_bit(x, y, ROWS as u32, COLS as u32);
        let index = index as usize;

        match color {
            Color::Black => {
                self.buff[index] &= !bit;
            }
            Color::White => {
                self.buff[index] |= bit;
            }
        }
    }

    fn update(&mut self) {
        Command::XAddress(0x0).execute(&mut self.interface);
        Command::YAddress(0x0).execute(&mut self.interface);
        self.interface.send_command(0x24);
        self.interface.send_data(self.buff);
        Command::UpdateDisplayOption2(0xC7).execute(&mut self.interface);
        Command::UpdateDisplay.execute(&mut self.interface);
        self.interface.busy_wait();
    }
}

#[app(device = stm32f1::stm32f103)]
const APP: () = {
    //static mut SLEEP: u32 = 0;

    #[init]
    fn init() {
        let device: stm32f1::stm32f103::Peripherals = device;
        let core: rtfm::Peripherals = core;
        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);

        let clocks = rcc
            .cfgr
            .sysclk(64.mhz())
            .pclk1(32.mhz())
            .freeze(&mut flash.acr);

        let mut delay = Delay::new(core.SYST, clocks);

        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);

        let reset = gpioa.pa1.into_push_pull_output(&mut gpioa.crl);
        let dc = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
        let cs = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
        let busy = gpioa.pa4;

        // I2C1 for htu21d
        let scl1 = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
        let sda1 = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

        let i2c_hum = I2c::i2c1(device.I2C1, (scl1, sda1), &mut afio.mapr, Mode::Standard{frequency: 100_000}, clocks, &mut rcc.apb1);
        let i2c_hum = blocking_i2c(i2c_hum, clocks, 1000,10,1000,1000);

        //let mut htu21df = Htu21df::new(i2c_hum).unwrap();
        //let hum = htu21df.get_humidity();

        let mut stdout = hio::hstdout().unwrap();
        //write!(stdout, "{:?}", hum).unwrap();

        // I2C2 for bmp180
        let scl2 = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
        let sda2 = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);

        let i2c_press = I2c::i2c2(device.I2C2, (scl2, sda2), Mode::Standard{frequency: 100_000}, clocks, &mut rcc.apb1);
        let i2c_press = blocking_i2c(i2c_press, clocks, 1000,10,1000,1000);

        let mut bmpx85 = Bmpx85::new(i2c_press);
        let chip = bmpx85.get_chip_id();

        write!(stdout, "{:?}", chip).unwrap();

        // SPI1 display
        let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
        let miso = gpioa.pa6;
        let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);

        let spi = Spi::spi1(
            device.SPI1,
            (sck, miso, mosi),
            &mut afio.mapr,
            MODE_0,
            1.mhz(),
            clocks,
            &mut rcc.apb2,
        );

        let mut controller = ssd1675::Interface::new(spi, cs, busy, dc, reset);
        let mut buffer = [255u8; ROWS as usize * COLS as usize / 8];
        let mut display = Display {
            interface: controller,
            buff: &mut buffer,
        };
        display.init(&mut delay);
        display.draw(
            ProFont24Point::render_str("It Works")
                .with_stroke(Some(Color::Black))
                .with_fill(Some(Color::White))
                .translate(Coord::new(1, -4))
                .into_iter(),
        );
        display.draw(
            ProFont9Point::render_str("Yes it does")
                .with_stroke(Some(Color::Black))
                .with_fill(Some(Color::White))
                .translate(Coord::new(1, 24))
                .into_iter(),
        );
        display.update();
    }

    #[idle]
    fn idle() -> ! {
        loop {
            // wait for an interrupt (sleep)
            wfi();
        }
    }
};

fn get_bit(x: u32, y: u32, width: u32, height: u32) -> (u32, u8) {
    (y / 8 + (height - 1 - x) * (width / 8), 0x80 >> (y % 8))
}

impl<'a, SPI, CS, BUSY, DC, RESET> Drawing<Color> for Display<'a, SPI, CS, BUSY, DC, RESET>
where
    SPI: embedded_hal::blocking::spi::Write<u8>,
    CS: embedded_hal::digital::OutputPin,
    BUSY: embedded_hal::digital::InputPin,
    DC: embedded_hal::digital::OutputPin,
    RESET: embedded_hal::digital::OutputPin,
{
    fn draw<T>(&mut self, item_pixels: T)
    where
        T: Iterator<Item = Pixel<Color>>,
    {
        for Pixel(UnsignedCoord(x, y), colour) in item_pixels {
            self.set_pixel(x, y, colour);
        }
    }
}
