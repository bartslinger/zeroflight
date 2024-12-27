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

        defmt::info!("Clock setup");
        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(168.MHz()).require_pll48clk().freeze();
        defmt::info!("SYSCLK: {}", clocks.sysclk().raw());
        defmt::info!("HCLK: {}", clocks.hclk().raw());
        defmt::info!("PCLK1: {}", clocks.pclk1().raw());
        defmt::info!("PCLK2: {}", clocks.pclk2().raw());
        assert!(clocks.is_pll48clk_valid());

        let mut delay = cx.core.SYST.delay(&clocks);

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
        if let Err(_) = res {
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
            100.kHz(),
            &clocks,
        );

        let mut cs = gpioa.pa4.into_push_pull_output();
        cs.set_high();
        // delay
        delay.delay_ms(1);

        defmt::info!("IMU initializing...");

        // disable power on accel and gyro for configuration (see datasheet 12.9)
        cs.set_low();
        let mut buf = [0x4E, 0x00];
        spi1.transfer_in_place(&mut buf).expect("IMU write failed");
        cs.set_high();
        // min 200us sleep recommended
        delay.delay_us(300);

        // configure the FIFO
        cs.set_low();
        let mut buf = [0x16, 0x80]; // FIFO_CONFIG STOP-on-full
        spi1.transfer_in_place(&mut buf).expect("IMU write failed");
        cs.set_high();
        delay.delay_us(300);

        cs.set_low();
        let mut buf = [0x5F, 0x07]; // FIFO_CONFIG1 enable temp, accel, gyro
        spi1.transfer_in_place(&mut buf).expect("IMU write failed");
        cs.set_high();
        delay.delay_us(300);

        cs.set_low();
        let mut buf = [0x4C, 0xE0]; // big Endian, count records, hold last sample
        spi1.transfer_in_place(&mut buf).expect("IMU write failed");
        cs.set_high();
        delay.delay_us(300);

        cs.set_low();
        let mut buf = [0x4B, 0x02]; // SIGNAL_PATH_RESET flush the FIFO
        spi1.transfer_in_place(&mut buf).expect("IMU write failed");
        cs.set_high();
        delay.delay_us(300);

        // // read power bits
        // cs.set_low();
        // let mut buf = [0x4E | 0x80, 0x00];
        // spi1.transfer_in_place(&mut buf).expect("IMU write failed");
        // cs.set_high();
        // defmt::info!("Power bits: {:02x}", buf[1]);
        // delay.delay_ms(15);

        // set accel range
        cs.set_low();
        let mut buf = [0x50, (0x00 << 5) | (0x06 & 0x0F)];
        spi1.transfer_in_place(&mut buf).expect("IMU write failed");
        cs.set_high();
        delay.delay_ms(15);

        // enable power on accel and gyro
        cs.set_low();
        let mut buf = [0x4E, 0x0F];
        spi1.transfer_in_place(&mut buf).expect("IMU write failed");
        cs.set_high();
        // min 200us sleep recommended
        delay.delay_us(300);

        let mut fifo_buffer = [0u8; 16 * 10 + 1];
        loop {
            // read fifo count
            cs.set_low();
            let mut buf = [0x2E | 0x80, 0x00, 0x00];
            spi1.transfer_in_place(&mut buf).expect("IMU write failed");
            cs.set_high();
            delay.delay_us(300);
            let fifo_count = u16::from_be_bytes([buf[1], buf[2]]);
            defmt::info!("FIFO count: {}", fifo_count);

            for _ in 0..fifo_count {
                // read fifo data
                cs.set_low();
                let mut buf = [
                    0x30 | 0x80,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                ];
                spi1.transfer_in_place(&mut buf).expect("IMU write failed");
                if fifo_count < 100 {
                    defmt::info!(
                        "FIFO data: {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}",
                        buf[0],
                        buf[1],
                        buf[2],
                        buf[3],
                        buf[4],
                        buf[5],
                        buf[6],
                        buf[7],
                        buf[8],
                        buf[9],
                        buf[10],
                        buf[11],
                        buf[12],
                        buf[13],
                        buf[14],
                        buf[15],
                        buf[16],
                    );
                }
                cs.set_high();
            }

            // cs.set_low();
            // let mut buf = [0x1D | 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
            // spi1.transfer_in_place(&mut buf).expect("IMU write failed");
            // cs.set_high();
            // let raw_temperature = i16::from_be_bytes([buf[1], buf[2]]);
            // let raw_acc_x = i16::from_be_bytes([buf[3], buf[4]]);
            // let raw_acc_y = i16::from_be_bytes([buf[5], buf[6]]);
            // let raw_acc_z = i16::from_be_bytes([buf[7], buf[8]]);
            // let acc_x = raw_acc_x as f32 * 9.80665 / 2048.0;
            // let acc_y = raw_acc_y as f32 * 9.80665 / 2048.0;
            // let acc_z = raw_acc_z as f32 * 9.80665 / 2048.0;
            //
            // let temperature_celsius = (raw_temperature as f32 / 132.48) + 25.0;
            // defmt::info!("Temperature: {}", temperature_celsius);
            // defmt::info!("Accel: ({}, {}, {})", acc_x, acc_y, acc_z);
            delay.delay_ms(5);
        }

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

        Mono::start(delay.release().release(), 168_000_000);
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
