#![no_std]
#![no_main]

extern crate panic_abort;
use byteorder::{BigEndian, ByteOrder};
use embedded_graphics::{
    prelude::{UnsignedCoord, *},
    Drawing,
};
use embedded_hal::{digital::v2::OutputPin, spi::MODE_0};
use rtfm::{app, export::wfi};
use ssd1675::{
    command::{BufCommand, Command, DataEntryMode, IncrementAxis},
    interface::{DisplayInterface, Interface},
};
use stm32f1xx_hal::{
    delay::Delay,
    prelude::*,
    spi::Spi,
    usb::{Peripheral, UsbBus, UsbBusType},
};
use usb_device::{bus, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

// Debug
//use cortex_m_semihosting::{debug, hprintln};

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

pub struct Display<'a, SPI, CS, BUSY, DC, RESET>
where
    SPI: embedded_hal::blocking::spi::Write<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    BUSY: embedded_hal::digital::v2::InputPin,
    DC: embedded_hal::digital::v2::OutputPin,
    RESET: embedded_hal::digital::v2::OutputPin,
{
    interface: Interface<SPI, CS, BUSY, DC, RESET>,
    buff: &'a mut [u8],
}

impl<'a, SPI, CS, BUSY, DC, RESET> Display<'a, SPI, CS, BUSY, DC, RESET>
where
    SPI: embedded_hal::blocking::spi::Write<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    BUSY: embedded_hal::digital::v2::InputPin,
    DC: embedded_hal::digital::v2::OutputPin,
    RESET: embedded_hal::digital::v2::OutputPin,
{
    fn init<D: embedded_hal::blocking::delay::DelayMs<u8>>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), <SPI as embedded_hal::blocking::spi::Write<u8>>::Error> {
        self.interface.reset(delay);
        Command::SoftReset.execute(&mut self.interface)?;
        self.interface.busy_wait();
        Command::AnalogBlockControl(ANALOG_BLOCK_CONTROL_MAGIC).execute(&mut self.interface)?;
        Command::DigitalBlockControl(DIGITAL_BLOCK_CONTROL_MAGIC).execute(&mut self.interface)?;
        Command::DriverOutputControl(0xF9, 0x00).execute(&mut self.interface)?;
        Command::DataEntryMode(
            DataEntryMode::IncrementYIncrementX,
            IncrementAxis::Horizontal,
        )
        .execute(&mut self.interface)?;
        Command::StartEndXPosition(0x0, 0x0F).execute(&mut self.interface)?; //0x0F-->(15+1)*8=128
        Command::StartEndYPosition(0x0, 0xF9).execute(&mut self.interface)?; //0xF9-->(249+1)=250
        Command::BorderWaveform(0x03).execute(&mut self.interface)?;
        Command::WriteVCOM(0x55).execute(&mut self.interface)?;
        Command::GateDrivingVoltage(0x15).execute(&mut self.interface)?;
        Command::SourceDrivingVoltage(0x41, 0xA8, 0x32).execute(&mut self.interface)?;
        Command::DummyLinePeriod(0x30).execute(&mut self.interface)?;
        Command::GateLineWidth(0x0A).execute(&mut self.interface)?;
        BufCommand::WriteLUT(&LUT).execute(&mut self.interface)?;
        self.interface.busy_wait();
        Ok(())
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

    fn update(&mut self) -> Result<(), <SPI as embedded_hal::blocking::spi::Write<u8>>::Error> {
        Command::XAddress(0x0).execute(&mut self.interface)?;
        Command::YAddress(0x0).execute(&mut self.interface)?;
        self.interface.send_command(0x24)?;
        self.interface.send_data(self.buff)?;
        Command::UpdateDisplayOption2(0xC7).execute(&mut self.interface)?;
        Command::UpdateDisplay.execute(&mut self.interface)?;
        self.interface.busy_wait();
        Ok(())
    }
}

pub type DISPLAY = Display<
    'static,
    stm32f1xx_hal::spi::Spi<
        stm32f1::stm32f103::SPI1,
        stm32f1xx_hal::spi::Spi1NoRemap,
        (
            stm32f1xx_hal::gpio::gpioa::PA5<
                stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::PushPull>,
            >,
            stm32f1xx_hal::gpio::gpioa::PA6<
                stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::Floating>,
            >,
            stm32f1xx_hal::gpio::gpioa::PA7<
                stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::PushPull>,
            >,
        ),
    >,
    stm32f1xx_hal::gpio::gpioa::PA3<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>>,
    stm32f1xx_hal::gpio::gpioa::PA4<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::Floating>>,
    stm32f1xx_hal::gpio::gpioa::PA2<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>>,
    stm32f1xx_hal::gpio::gpioa::PA1<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>>,
>;
#[app(device = stm32f1::stm32f103, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: SerialPort<'static, UsbBusType>,
        display: DISPLAY,
        #[init([255u8; ROWS as usize * COLS as usize / 8])]
        buffer: [u8; ROWS as usize * COLS as usize / 8],
        #[init(0)]
        index: u32,
        #[init(0)]
        size: u32,
    }

    #[init(resources = [buffer])]
    fn init(cx: init::Context) -> init::LateResources {
        let cp = cortex_m::Peripherals::take().unwrap();
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let mut afio = cx.device.AFIO.constrain(&mut rcc.apb2);
        static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;

        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(48.mhz())
            .pclk1(24.mhz())
            .freeze(&mut flash.acr);

        assert!(clocks.usbclk_valid());

        let mut delay = Delay::new(cp.SYST, clocks);

        let mut gpioa = cx.device.GPIOA.split(&mut rcc.apb2);

        // BluePill board has a pull-up resistor on the D+ line.
        // Pull the D+ pin down to send a RESET condition to the USB bus.
        // This forced reset is needed only for development, without it host
        // will not reset your device when you upload new firmware.
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low().unwrap();
        cortex_m::asm::delay(clocks.sysclk().0 / 100);

        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);
        let usb = Peripheral {
            usb: cx.device.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        let reset = gpioa.pa1.into_push_pull_output(&mut gpioa.crl);
        let dc = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
        let cs = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
        let busy = gpioa.pa4;

        // SPI1 display
        let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
        let miso = gpioa.pa6;
        let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);

        let spi = Spi::spi1(
            cx.device.SPI1,
            (sck, miso, mosi),
            &mut afio.mapr,
            MODE_0,
            1.mhz(),
            clocks,
            &mut rcc.apb2,
        );

        let controller = ssd1675::Interface::new(spi, cs, busy, dc, reset);
        let mut display = Display {
            interface: controller,
            buff: cx.resources.buffer,
        };

        if let Ok(_) = display.init(&mut delay) {};

        if let Ok(_) = display.update() {};

        unsafe {
            USB_BUS = Some(UsbBus::new(usb));

            let serial = SerialPort::new(USB_BUS.as_ref().unwrap());

            let usb_dev =
                UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
                    .manufacturer("Fake company")
                    .product("Serial port")
                    .serial_number("TEST")
                    .device_class(USB_CLASS_CDC)
                    .build();

            init::LateResources {
                usb_dev,
                serial,
                display,
            }
        }
    }

    #[task(binds = USB_HP_CAN_TX, resources = [usb_dev, serial, display, index, size])]
    fn usb_tx(mut cx: usb_tx::Context) {
        usb_poll(
            &mut cx.resources.usb_dev,
            &mut cx.resources.serial,
            &mut cx.resources.display,
            &mut cx.resources.index,
            &mut cx.resources.size,
        );
    }

    #[task(binds = USB_LP_CAN_RX0, resources = [usb_dev, serial, display, index, size])]
    fn usb_rx0(mut cx: usb_rx0::Context) {
        usb_poll(
            &mut cx.resources.usb_dev,
            &mut cx.resources.serial,
            &mut cx.resources.display,
            &mut cx.resources.index,
            &mut cx.resources.size,
        );
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            // wait for an interrupt (sleep)
            wfi();
        }
    }
};

fn usb_poll<B: bus::UsbBus>(
    usb_dev: &mut UsbDevice<'static, B>,
    serial: &mut SerialPort<'static, B>,
    display: &mut DISPLAY,
    index: &mut u32,
    size: &mut u32,
) {
    while usb_dev.poll(&mut [serial]) {
        let mut buff = [255u8; 32];
        match serial.read(&mut buff) {
            Ok(count) if count > 0 => match size {
                0 => {
                    let new_size = BigEndian::read_u32(&buff[0..4]);
                    *size = new_size;
                }
                _ => {
                    for (i, byte) in buff[0..count].iter().enumerate() {
                        display.buff[*index as usize + i] = *byte;
                    }
                    *index += count as u32;
                }
            },
            _ => {}
        }
    }
    if *size > 0 && *index >= *size {
        *size = 0;
        *index = 0;
        display.update().unwrap();
    }
}

fn get_bit(x: u32, y: u32, width: u32, height: u32) -> (u32, u8) {
    (y / 8 + (height - 1 - x) * (width / 8), 0x80 >> (y % 8))
}

impl<'a, SPI, CS, BUSY, DC, RESET> Drawing<Color> for Display<'a, SPI, CS, BUSY, DC, RESET>
where
    SPI: embedded_hal::blocking::spi::Write<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    BUSY: embedded_hal::digital::v2::InputPin,
    DC: embedded_hal::digital::v2::OutputPin,
    RESET: embedded_hal::digital::v2::OutputPin,
{
    fn draw<T>(&mut self, item_pixels: T)
    where
        T: IntoIterator<Item = Pixel<Color>>,
    {
        for Pixel(UnsignedCoord(x, y), colour) in item_pixels {
            self.set_pixel(x, y, colour);
        }
    }
}
