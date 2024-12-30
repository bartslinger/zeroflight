#![no_main]
#![no_std]
//#![deny(warnings)]
#![deny(unsafe_code)]
//#![deny(missing_docs)]

mod blink;
mod icm42688p;
mod imu;
mod usb;

use defmt_rtt as _;
use panic_halt as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART1, USART2])]
mod app {
    use crate::blink::blink;
    use crate::imu::imu_rx_handler;
    use crate::usb::usb_tx;
    use rtic_monotonics::systick::prelude::*;
    use stm32f4xx_hal::gpio;

    systick_monotonic!(Mono, 1000);

    #[shared]
    struct Shared {
        usb_dev: usb_device::device::UsbDevice<'static, stm32f4xx_hal::otg_fs::UsbBusType>,
        serial: usbd_serial::SerialPort<'static, stm32f4xx_hal::otg_fs::UsbBusType>,
    }

    #[local]
    struct Local {
        dwt: cortex_m::peripheral::DWT,
        icm42688p_dma_context:
            crate::icm42688p::Icm42688pDmaContext<gpio::PA4<gpio::Output<gpio::PushPull>>>,
        prev_fifo_count: u16,
        imu_data_sender: rtic_sync::channel::Sender<'static, [u8; 16], 5>,
    }

    #[init(local = [
        EP_MEMORY: [u32; 1024] = [0; 1024],
        USB_BUS: Option<usb_device::bus::UsbBusAllocator<stm32f4xx_hal::otg_fs::UsbBusType>> =
            None,
        TX_BUFFER_1: [u8; 129] = [0x00; 129],
        TX_BUFFER_2: [u8; 129] = [0x00; 129],
        RX_BUFFER_1: [u8; 129] = [0x00; 129],
        RX_BUFFER_2: [u8; 129] = [0x00; 129],
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        use stm32f4xx_hal::prelude::*; // for .freeze() and constrain()
        defmt::info!("init");

        let (imu_data_sender, imu_data_receiver) = rtic_sync::make_channel!([u8; 16], 5);
        imu_handler::spawn(imu_data_receiver).ok();

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

        let spi1 = stm32f4xx_hal::spi::Spi::new(
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
        let mut icm42688p = crate::icm42688p::Icm42688p::new(spi1, cs);
        icm42688p.init(&mut delay);

        // -----------------------------------------------------------------------------------------
        let icm42688p_dma_context = icm42688p.start_dma(
            dma2.3,
            dma2.0,
            cx.local.TX_BUFFER_1,
            cx.local.RX_BUFFER_1,
            cx.local.TX_BUFFER_2,
            cx.local.RX_BUFFER_2,
        );

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
        (
            Shared { usb_dev, serial },
            Local {
                dwt,
                icm42688p_dma_context,
                prev_fifo_count: 0,
                imu_data_sender,
            },
        )
    }

    #[idle(local = [dwt])]
    fn idle(cx: idle::Context) -> ! {
        let dwt = cx.local.dwt;
        blink::spawn().ok();

        let mut prev_cyc_cnt = dwt.cyccnt.read();
        let mut counter = 0;
        loop {
            // defmt::info!("loop");
            cortex_m::asm::nop();
            counter += 1;
            if counter >= 16_800_000 {
                let cyc_cnt = dwt.cyccnt.read();
                let time_passed = cyc_cnt.wrapping_sub(prev_cyc_cnt);
                let idle_time = 16_800_000_f32 * 10.;
                let _ratio = 100. * (idle_time / time_passed as f32);

                // defmt::info!("Idle: {}", ratio);
                prev_cyc_cnt = cyc_cnt;
                counter = 0;
            }
        }
    }

    #[task(priority = 2)]
    async fn imu_handler(
        _cx: imu_handler::Context,
        mut imu_data_receiver: rtic_sync::channel::Receiver<'static, [u8; 16], 5>,
    ) {
        defmt::info!("imu handler spawned");
        while let Ok(buf) = imu_data_receiver.recv().await {
            let raw_temperature = buf[13];
            let raw_acc_x = i16::from_be_bytes([buf[1], buf[2]]);
            let raw_acc_y = i16::from_be_bytes([buf[3], buf[4]]);
            let raw_acc_z = i16::from_be_bytes([buf[5], buf[6]]);
            let acc_x = raw_acc_x as f32 * 9.80665 / 2048.0;
            let acc_y = -raw_acc_y as f32 * 9.80665 / 2048.0;
            let acc_z = raw_acc_z as f32 * 9.80665 / 2048.0;

            let temperature_celsius = (raw_temperature as f32 / 2.07) + 25.0;
            defmt::info!(
                "T: {} A: {} {} {}",
                temperature_celsius,
                acc_x,
                acc_y,
                acc_z
            );
        }
    }

    extern "Rust" {
        #[task(priority = 1)]
        async fn blink(cx: blink::Context);

        #[task(binds = OTG_FS, shared = [usb_dev, serial], priority = 1)]
        fn usb_tx(cx: usb_tx::Context);

        #[task(
            binds = DMA2_STREAM0,
            shared = [],
            local = [icm42688p_dma_context, prev_fifo_count, imu_data_sender],
            priority = 5
        )]
        fn imu_rx_handler(cx: imu_rx_handler::Context);
    }
}
