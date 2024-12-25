#![no_main]
#![no_std]
//#![deny(warnings)]
#![deny(unsafe_code)]
//#![deny(missing_docs)]

use defmt_rtt as _;
use panic_halt as _;

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [USART1])]
mod app {
    use stm32f1xx_hal::gpio::{self, Output, PushPull};
    // use cortex_m_semihosting::{debug, hprintln};
    use rtic_monotonics::systick::prelude::*;

    systick_monotonic!(Mono, 1000);

    #[shared]
    struct Shared {
        usb_dev: usb_device::device::UsbDevice<'static, stm32f1xx_hal::usb::UsbBusType>,
        serial: usbd_serial::SerialPort<'static, stm32f1xx_hal::usb::UsbBusType>,
    }

    #[local]
    struct Local {
        led: gpio::PC13<Output<PushPull>>,
    }

    #[init(local = [
    usb_bus: Option<
            usb_device::bus::UsbBusAllocator<
                stm32f1xx_hal::usb::UsbBus<stm32f1xx_hal::usb::Peripheral>,
            >,
        > = None
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        use stm32f1xx_hal::prelude::*;

        Mono::start(cx.core.SYST, 72_000_000);
        defmt::info!("clock setup");
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz()) // Use an external 8 MHz crystal oscillator
            .sysclk(72.MHz()) // Set system clock to 72 MHz
            .pclk1(36.MHz()) // Set APB1 clock (max 36 MHz)
            .pclk2(72.MHz()) // Set APB2 clock (max 72 MHz)
            .freeze(&mut flash.acr);

        assert!(clocks.usbclk_valid());

        // Configure LED
        let mut gpioc = cx.device.GPIOC.split();
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_low();

        // Configure USB as CDC-ACM
        let mut gpioa = cx.device.GPIOA.split();
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low();
        cortex_m::asm::delay(clocks.sysclk().raw() / 100);

        let usb = stm32f1xx_hal::usb::Peripheral {
            usb: cx.device.USB,
            pin_dm: gpioa.pa11,
            pin_dp: usb_dp.into_floating_input(&mut gpioa.crh),
        };

        let usb_bus = stm32f1xx_hal::usb::UsbBus::new(usb);
        *cx.local.usb_bus = Some(usb_bus);
        let usb_bus_reference = cx.local.usb_bus.as_ref().unwrap();

        let serial = usbd_serial::SerialPort::new(usb_bus_reference);
        let usb_dev = usb_device::device::UsbDeviceBuilder::new(
            usb_bus_reference,
            usb_device::device::UsbVidPid(0x16c0, 0x27dd),
        )
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

        (Shared { usb_dev, serial }, Local { led })
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        blink::spawn().ok();

        loop {
            // defmt::info!("loop");
            cortex_m::asm::nop();
        }
    }

    #[task(priority = 2, local = [led])]
    async fn blink(cx: blink::Context) {
        defmt::info!("blink task");
        let mut instant = Mono::now();
        loop {
            instant += 1000.millis();
            Mono::delay_until(instant).await;
            cx.local.led.set_high();
            let now = Mono::now();
            defmt::info!("LED off at {:?}", now.ticks());

            instant += 1000.millis();
            Mono::delay_until(instant).await;
            cx.local.led.set_low();
            let now = Mono::now();
            defmt::info!("LED on  at {:?}", now.ticks());
        }
    }

    #[task(binds = USB_HP_CAN_TX, shared = [usb_dev, serial], priority = 1)]
    fn usb_tx(cx: usb_tx::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;

        (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
            super::usb_poll(usb_dev, serial);
        });
    }

    #[task(binds = USB_LP_CAN_RX0, shared = [usb_dev, serial], priority = 1)]
    fn usb_rx0(cx: usb_rx0::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;

        (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
            super::usb_poll(usb_dev, serial);
        });
    }
}

fn usb_poll<B: usb_device::bus::UsbBus>(
    usb_dev: &mut usb_device::prelude::UsbDevice<'static, B>,
    serial: &mut usbd_serial::SerialPort<'static, B>,
) {
    if !usb_dev.poll(&mut [serial]) {
        return;
    }

    let mut buf = [0u8; 8];

    match serial.read(&mut buf) {
        Ok(count) if count > 0 => {
            // Echo back in upper case
            for c in buf[0..count].iter_mut() {
                if 0x61 <= *c && *c <= 0x7a {
                    *c &= !0x20;
                }
            }

            serial.write(&buf[0..count]).ok();
            // cortex_m::asm::delay(7_000_000); // for testing priority
        }
        _ => {}
    }
}
