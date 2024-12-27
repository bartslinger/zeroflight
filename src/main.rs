#![no_main]
#![no_std]
//#![deny(warnings)]
#![deny(unsafe_code)]
//#![deny(missing_docs)]

use defmt_rtt as _;
use panic_halt as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART1])]
mod app {
    // use cortex_m_semihosting::{debug, hprintln};
    use rtic_monotonics::systick::prelude::*;

    systick_monotonic!(Mono, 1000);

    #[shared]
    struct Shared {
        usb_dev: usb_device::device::UsbDevice<'static, stm32f4xx_hal::otg_fs::UsbBusType>,
        serial: usbd_serial::SerialPort<'static, stm32f4xx_hal::otg_fs::UsbBusType>,
    }

    #[local]
    struct Local {
        // led: gpio::PC13<Output<PushPull>>,
        dwt: cortex_m::peripheral::DWT,
    }

    #[init(local = [
        EP_MEMORY: [u32; 1024] = [0; 1024],
        USB_BUS: Option<usb_device::bus::UsbBusAllocator<stm32f4xx_hal::otg_fs::UsbBusType>> =
            None
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        use stm32f4xx_hal::prelude::*; // for .freeze() and constrain()
        defmt::info!("init");

        Mono::start(cx.core.SYST, 168_000_000);
        defmt::info!("Clock setup");
        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(168.MHz()).require_pll48clk().freeze();
        defmt::info!("SYSCLK: {}", clocks.sysclk().raw());
        defmt::info!("HCLK: {}", clocks.hclk().raw());
        defmt::info!("PCLK1: {}", clocks.pclk1().raw());
        defmt::info!("PCLK2: {}", clocks.pclk2().raw());
        assert!(clocks.is_pll48clk_valid());

        // -----------------------------------------------------------------------------------------
        // Enable cycle counter for counting clock ticks
        let mut dwt = cx.core.DWT;
        dwt.enable_cycle_counter();
        let mut dcb = cx.core.DCB;
        dcb.enable_trace();

        // -----------------------------------------------------------------------------------------
        // DMA
        let _dma1 = stm32f4xx_hal::dma::StreamsTuple::new(cx.device.DMA1);

        // -----------------------------------------------------------------------------------------
        // Configure I2C1
        let gpiob = cx.device.GPIOB.split();
        let scl = gpiob.pb8.into_alternate_open_drain();
        let sda = gpiob.pb9.into_alternate_open_drain();
        let mut i2c = stm32f4xx_hal::i2c::I2c::new(
            cx.device.I2C1,
            (scl, sda),
            stm32f4xx_hal::i2c::Mode::from(400.kHz()),
            &clocks,
        );

        // -----------------------------------------------------------------------------------------
        // Scan I2C
        defmt::info!("Start I2C scan...");

        const VALID_ADDR_RANGE: core::ops::Range<u8> = 0x08..0x78;

        for addr in 0x00_u8..0x80 {
            let byte: [u8; 1] = [0; 1];
            if VALID_ADDR_RANGE.contains(&addr) && i2c.write(addr, &byte).is_ok() {
                defmt::info!("Found device at {:02x}", addr);
            }
        }

        // -----------------------------------------------------------------------------------------
        // Initialize SPL06
        // Not complete
        let start = dwt.cyccnt.read();

        let mut buf = [0];
        // who am i?
        let res = i2c.write_read(0x76_u8, &[0x0D], &mut buf);
        if let Err(e) = res {
            defmt::error!("I2C write_read error");
        }

        let end = dwt.cyccnt.read();
        let diff = end.wrapping_sub(start);
        defmt::info!("Baro initialization cycle count: {}", diff);

        defmt::info!("WHO_AM_I: {:02x}", buf[0]);

        // -----------------------------------------------------------------------------------------
        // Configure ICM-42688-P on SPI
        let gpioa = cx.device.GPIOA.split();

        let sck = gpioa.pa5.into_alternate();

        let miso = gpioa.pa6.into_alternate();
        let mosi = gpioa.pa7.into_alternate();

        let mut spi1 = stm32f4xx_hal::spi::Spi::new(
            cx.device.SPI1,
            (sck, miso, mosi),
            stm32f4xx_hal::spi::Mode {
                polarity: stm32f4xx_hal::spi::Polarity::IdleHigh,
                phase: stm32f4xx_hal::spi::Phase::CaptureOnSecondTransition,
            },
            1.MHz(),
            &clocks,
        );

        let mut cs = gpioa.pa4.into_push_pull_output();
        cs.set_high(); // deactivate chip select
        cortex_m::asm::delay(clocks.sysclk().raw() / 100);
        let start = dwt.cyccnt.read();
        cs.set_low();
        let mut buf = [0x75 | 0x80, 0x00];
        let result = spi1.transfer_in_place(&mut buf);
        cs.set_high(); // deactivate chip select
        let end = dwt.cyccnt.read();
        let diff = end.wrapping_sub(start);
        if let Err(e) = result {
            defmt::error!("SPI transfer error");
        }
        defmt::info!("WHO_AM_I: {:02x} {:02x}", buf[0], buf[1]);
        defmt::info!("IMU initialization cycle count: {}", diff);

        // -----------------------------------------------------------------------------------------
        // Configure USB as CDC-ACM
        let mut usb_dp = gpioa.pa12.into_push_pull_output();
        usb_dp.set_low();
        cortex_m::asm::delay(clocks.sysclk().raw() / 100);

        let usb = stm32f4xx_hal::otg_fs::USB {
            usb_global: cx.device.OTG_FS_GLOBAL,
            usb_device: cx.device.OTG_FS_DEVICE,
            usb_pwrclk: cx.device.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into(),
            pin_dp: usb_dp.into(),
            hclk: clocks.hclk(),
        };

        let usb_bus = stm32f4xx_hal::otg_fs::UsbBus::new(usb, cx.local.EP_MEMORY);
        *cx.local.USB_BUS = Some(usb_bus);

        let usb_bus_reference = cx.local.USB_BUS.as_ref().unwrap();
        let serial = usbd_serial::SerialPort::new(usb_bus_reference);
        let usb_dev = usb_device::device::UsbDeviceBuilder::new(
            usb_bus_reference,
            usb_device::device::UsbVidPid(0x16c0, 0x27dd),
        )
        .device_class(usbd_serial::USB_CLASS_CDC)
        .strings(&[usb_device::device::StringDescriptors::default()
            .manufacturer("ZeroFlight")
            .product("SpeedyBee F405 Wing")
            .serial_number("TEST")])
        .unwrap()
        .build();

        (Shared { usb_dev, serial }, Local { dwt })
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        blink::spawn().ok();

        loop {
            // defmt::info!("loop");
            cortex_m::asm::nop();
        }
    }

    #[task(priority = 2)]
    async fn blink(_cx: blink::Context) {
        defmt::info!("blink task");
        let mut instant = Mono::now();
        loop {
            instant += 1000.millis();
            Mono::delay_until(instant).await;
            // cx.local.led.set_high();
            // let now = Mono::now();
            // defmt::info!("LED off at {:?}", now.ticks());

            instant += 1000.millis();
            Mono::delay_until(instant).await;
            // cx.local.led.set_low();
            // let now = Mono::now();
            // defmt::info!("LED on  at {:?}", now.ticks());
        }
    }

    #[task(binds = OTG_FS, local = [dwt], shared = [usb_dev, serial], priority = 1)]
    fn usb_tx(cx: usb_tx::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;

        (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
            super::usb_poll(usb_dev, serial);
        });
    }
}

fn usb_poll<B: usb_device::bus::UsbBus>(
    usb_dev: &mut usb_device::device::UsbDevice<'static, B>,
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
            // cortex_m::asm::delay(20_000_000); // for testing priority
        }
        _ => {}
    }
}
