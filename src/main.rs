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
    use stm32f4xx_hal::dma::traits::StreamISR;
    use stm32f4xx_hal::hal_02::digital::v2::OutputPin;
    use stm32f4xx_hal::{ClearFlags, Listen, ReadFlags};

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
            None,
        READ_WHO_AM_I: [u8; 2] = [0x75 | 0x80, 0x00],
        ANSWER: [u8; 2] = [0xAA, 0xAA],
        TX_BUFFER_1: [u8; 2] = [0x75 | 0x80, 0x00],
        TX_BUFFER_2: [u8; 2] = [0x75 | 0x80, 0x00],
        RX_BUFFER_1: [u8; 2] = [0xAA, 0xBB],
        RX_BUFFER_2: [u8; 2] = [0xAA, 0xBB],
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
        let dma2 = stm32f4xx_hal::dma::StreamsTuple::new(cx.device.DMA2);

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
            1.MHz(),
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

        // try a two-step who-am-i
        let start = dwt.cyccnt.read();
        cs.set_low();
        let mut buf = [0x75 | 0x80];
        spi1.transfer_in_place(&mut buf).expect("IMU write failed");
        defmt::info!("was 0x75: {:02x}", buf[0]);
        let mut buf = [0x00];
        spi1.transfer_in_place(&mut buf).expect("IMU write failed");
        defmt::info!("who am i: {:02x}", buf[0]);
        cs.set_high();
        let end = dwt.cyccnt.read();
        let diff = end.wrapping_sub(start);
        defmt::info!("SPI read cycle count: {}", diff);

        // try a who-am-i with interrupts
        let start = dwt.cyccnt.read();
        cs.set_low();
        let mut buf = [0x75 | 0x80, 0x00];
        // spi1.listen(stm32f4xx_hal::spi::Event::RxNotEmpty | stm32f4xx_hal::spi::Event::TxEmpty);

        // try a who-am-i with dma
        let (tx, rx) = spi1.use_dma().txrx();

        let tx_stream = dma2.3;
        let rx_stream = dma2.0;

        let mut rx_transfer = stm32f4xx_hal::dma::Transfer::init_peripheral_to_memory(
            rx_stream,
            rx,
            cx.local.RX_BUFFER_1,
            None,
            stm32f4xx_hal::dma::config::DmaConfig::default().memory_increment(true),
        );
        let mut tx_transfer = stm32f4xx_hal::dma::Transfer::init_memory_to_peripheral(
            tx_stream,
            tx,
            cx.local.TX_BUFFER_1,
            None,
            stm32f4xx_hal::dma::config::DmaConfig::default().memory_increment(true),
        );

        let start = dwt.cyccnt.read();
        cs.set_low();
        // starting rx_transfer before tx_transfer seems more robust
        // (works with even large delay between the two calls)
        rx_transfer.start(|_rx| {
            defmt::info!("rx transfer started");
        });
        // delay.delay_ms(1000);
        tx_transfer.start(|_tx| {
            defmt::info!("tx transfer started");
        });

        let end = dwt.cyccnt.read();
        let diff = end.wrapping_sub(start);
        defmt::info!("DMA SPI start cycle count: {}", diff);

        rx_transfer.wait();
        let flags = tx_transfer.flags();
        if flags.is_transfer_complete() {
            defmt::info!("transfer complete");
        } else {
            defmt::info!("transfer not complete");
        }
        let flags = rx_transfer.flags();
        if flags.is_transfer_complete() {
            defmt::info!("transfer complete");
        } else {
            defmt::info!("transfer not complete");
        }
        // let (a, b, c, d) = rx_transfer.release();
        // defmt::info!("ok got {:02x}", c[1]);
        cs.set_high();
        delay.delay_us(1);

        cs.set_low();
        // send another DMA transfer by swapping the buffers
        let (prev_tx_buffer, _) = tx_transfer
            .next_transfer(cx.local.TX_BUFFER_2)
            .expect("no next");
        let (prev_rx_buffer, _) = rx_transfer
            .next_transfer(cx.local.RX_BUFFER_2)
            .expect("no next");

        defmt::info!(
            "prev tx buffer: {:02x} {:02x}",
            prev_tx_buffer[0],
            prev_tx_buffer[1]
        );
        defmt::info!(
            "prev rx buffer: {:02x} {:02x}",
            prev_rx_buffer[0],
            prev_rx_buffer[1],
        );

        rx_transfer.wait();
        cs.set_high();
        let mut ready_tx_buffer = prev_tx_buffer;
        let mut ready_rx_buffer = prev_rx_buffer;
        loop {
            delay.delay_ms(500);
            delay.delay_us(1);

            cs.set_low();
            let (prev_tx_buffer, _) = tx_transfer.next_transfer(ready_tx_buffer).expect("no next");
            let (prev_rx_buffer, _) = rx_transfer.next_transfer(ready_rx_buffer).expect("no next");

            defmt::info!(
                "prev tx buffer: {:02x} {:02x}",
                prev_tx_buffer[0],
                prev_tx_buffer[1]
            );
            defmt::info!(
                "prev rx buffer: {:02x} {:02x}",
                prev_rx_buffer[0],
                prev_rx_buffer[1],
            );

            rx_transfer.wait();
            cs.set_high();
            ready_tx_buffer = prev_tx_buffer;
            ready_rx_buffer = prev_rx_buffer;
        }

        // let mut total_samples = 0;
        // let mut fifo_buffer = [0u8; 16 * 10 + 1];
        // loop {
        //     delay.delay_us(1);
        //     // read fifo count
        //     cs.set_low();
        //     let mut buf = [0x2E | 0x80, 0x00, 0x00];
        //     spi1.transfer_in_place(&mut buf).expect("IMU write failed");
        //     cs.set_high();
        //     delay.delay_us(10);
        //     let fifo_count = u16::from_be_bytes([buf[1], buf[2]]);
        //
        //     let fifo_read_count = fifo_count.min(10);
        //     for _ in 0..fifo_read_count {
        //         total_samples += 1;
        //         if total_samples % 1000 == 0 {
        //             defmt::info!("Total samples: {}", total_samples);
        //         }
        //     }
        //     // defmt::info!("FIFO count: {}, read {}", fifo_count, fifo_read_count);
        //     let fifo_reg_read = 0x30 | 0x80;
        //     // read fifo data
        //     if fifo_read_count == 0 {
        //         continue;
        //     }
        //     cs.set_low();
        //     let mut buf = &mut fifo_buffer[..(16 * fifo_read_count as usize + 1)];
        //     buf[0] = fifo_reg_read;
        //     spi1.transfer_in_place(&mut buf).expect("IMU write failed");
        //     cs.set_high();
        //
        //     if fifo_read_count < 0 {
        //         defmt::info!(
        //             "FIFO data: {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}",
        //             buf[0],
        //             buf[1],
        //             buf[2],
        //             buf[3],
        //             buf[4],
        //             buf[5],
        //             buf[6],
        //             buf[7],
        //             buf[8],
        //             buf[9],
        //             buf[10],
        //             buf[11],
        //             buf[12],
        //             buf[13],
        //             buf[14],
        //             buf[15],
        //             buf[16],
        //         );
        //     }
        //
        //     // cs.set_low();
        //     // let mut buf = [0x1D | 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        //     // spi1.transfer_in_place(&mut buf).expect("IMU write failed");
        //     // cs.set_high();
        //     // let raw_temperature = i16::from_be_bytes([buf[1], buf[2]]);
        //     // let raw_acc_x = i16::from_be_bytes([buf[3], buf[4]]);
        //     // let raw_acc_y = i16::from_be_bytes([buf[5], buf[6]]);
        //     // let raw_acc_z = i16::from_be_bytes([buf[7], buf[8]]);
        //     // let acc_x = raw_acc_x as f32 * 9.80665 / 2048.0;
        //     // let acc_y = raw_acc_y as f32 * 9.80665 / 2048.0;
        //     // let acc_z = raw_acc_z as f32 * 9.80665 / 2048.0;
        //     //
        //     // let temperature_celsius = (raw_temperature as f32 / 132.48) + 25.0;
        //     // defmt::info!("Temperature: {}", temperature_celsius);
        //     // defmt::info!("Accel: ({}, {}, {})", acc_x, acc_y, acc_z);
        // }

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

    #[task(binds = SPI1, priority = 2)]
    fn spi_interrupt(cx: spi_interrupt::Context) {
        defmt::info!("SPI interrupt");
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
