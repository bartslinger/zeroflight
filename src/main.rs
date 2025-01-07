#![no_main]
#![no_std]
//#![deny(warnings)]
#![deny(unsafe_code)]
//#![deny(missing_docs)]

mod blink;
mod control;
mod controller;
mod crsf;
mod icm42688p;
mod imu;
mod pwm_output;
mod usb;

mod boards;

use defmt_rtt as _;
use heapless::box_pool;
use panic_halt as _;

struct OutputCommand {
    armed: bool,
    roll: u16,
    pitch: u16,
    throttle: u16,
    #[allow(unused)]
    yaw: u16,
}

// Declare a pool
box_pool!(IMUDATAPOOL: [u8; 16]);

#[rtic::app(device = board, dispatchers = [OTG_HS_EP1_OUT, OTG_HS_EP1_IN, OTG_HS_WKUP, OTG_HS])]
mod app {
    use crate::blink::blink;
    use crate::boards::board::{self, Board, PwmOutputs};
    use crate::control::control_task;
    use crate::crsf::crsf_parser;
    use crate::crsf::usart1_irq;
    use crate::crsf::RcState;
    use crate::imu::imu_rx_irq;
    use crate::imu::{imu_handler, AhrsState};
    use crate::pwm_output::pwm_output_task;
    use crate::usb::usb_tx;
    use crate::IMUDATAPOOL;
    use core::sync::atomic::AtomicBool;
    use heapless::pool::boxed::{Box, BoxBlock};
    use rtic_monotonics::systick::prelude::*;

    systick_monotonic!(Mono, 1000);

    pub struct Flags {
        pub reset_ahrs: AtomicBool,
    }

    #[shared]
    struct Shared {
        usb_dev: usb_device::device::UsbDevice<'static, stm32f4xx_hal::otg_fs::UsbBusType>,
        serial: usbd_serial::SerialPort<'static, stm32f4xx_hal::otg_fs::UsbBusType>,
        flags: Flags,
    }

    #[local]
    struct Local {
        dwt: cortex_m::peripheral::DWT,
        icm42688p_dma_context: crate::icm42688p::Icm42688pDmaContext,
        prev_fifo_count: u16,
        imu_data_sender: rtic_sync::channel::Sender<'static, Box<IMUDATAPOOL>, 1>,
        crsf_serial: board::CrsfSerial,
        crsf_data_sender: rtic_sync::channel::Sender<'static, u8, 64>,
        pwm_outputs: PwmOutputs,
    }

    #[init(local = [
        EP_MEMORY: [u32; 1024] = [0; 1024],
        USB_BUS: Option<usb_device::bus::UsbBusAllocator<stm32f4xx_hal::otg_fs::UsbBusType>> =
            None,
        TX_BUFFER_1: [u8; 129] = [0x00; 129],
        TX_BUFFER_2: [u8; 129] = [0x00; 129],
        RX_BUFFER_1: [u8; 129] = [0x00; 129],
        RX_BUFFER_2: [u8; 129] = [0x00; 129],
        IMU_DATA_CHANNEL_MEMORY: [BoxBlock<[u8; 16]>; 2] = [const { BoxBlock::new() }; 2],
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        use stm32f4xx_hal::prelude::*; // for .freeze() and constrain()
        let board = Board::from(cx.device);

        defmt::info!("init");

        for block in cx.local.IMU_DATA_CHANNEL_MEMORY {
            IMUDATAPOOL.manage(block);
        }

        let (imu_data_sender, imu_data_receiver) = rtic_sync::make_channel!(Box<IMUDATAPOOL>, 1);
        let (crsf_data_sender, crsf_data_receiver) = rtic_sync::make_channel!(u8, 64);
        let (rc_state_sender, rc_state_receiver) = rtic_sync::make_channel!(RcState, 1);
        let (pwm_output_sender, pwm_output_receiver) =
            rtic_sync::make_channel!(crate::OutputCommand, 1);
        let (ahrs_state_sender, ahrs_state_receiver) = rtic_sync::make_channel!(AhrsState, 1);

        let mut delay = cx.core.SYST.delay(&board.clocks);

        // -----------------------------------------------------------------------------------------
        // Enable cycle counter for counting clock ticks
        let mut dwt = cx.core.DWT;
        dwt.enable_cycle_counter();
        let mut dcb = cx.core.DCB;
        dcb.enable_trace();

        // -----------------------------------------------------------------------------------------
        // DMA
        let dma2 = stm32f4xx_hal::dma::StreamsTuple::new(board.DMA2);

        // -----------------------------------------------------------------------------------------
        // Scan I2C
        defmt::info!("Start I2C scan...");

        let mut i2c1 = stm32f4xx_hal::i2c::I2c::new(
            board.I2C1,
            (
                board.i2c1.scl.into_open_drain_output(),
                board.i2c1.sda.into_open_drain_output(),
            ),
            stm32f4xx_hal::i2c::Mode::from(400.kHz()),
            &board.clocks,
        );

        const VALID_ADDR_RANGE: core::ops::Range<u8> = 0x08..0x78;

        for addr in 0x00_u8..0x80 {
            let byte: [u8; 1] = [0; 1];
            if VALID_ADDR_RANGE.contains(&addr) && i2c1.write(addr, &byte).is_ok() {
                defmt::info!("Found device at {:02x}", addr);
            }
        }

        // -----------------------------------------------------------------------------------------
        // Initialize SPL06
        // Not complete
        let start = dwt.cyccnt.read();

        let mut buf = [0];
        // who am i?
        let res = i2c1.write_read(0x76_u8, &[0x0D], &mut buf);
        if let Err(_) = res {
            defmt::error!("I2C write_read error");
        }

        let end = dwt.cyccnt.read();
        let diff = end.wrapping_sub(start);
        defmt::info!("Baro initialization cycle count: {}", diff);

        defmt::info!("WHO_AM_I: {:02x}", buf[0]);

        // -----------------------------------------------------------------------------------------
        // Configure ICM-42688-P on SPI

        let spi1 = stm32f4xx_hal::spi::Spi::new(
            board.SPI1,
            (board.spi1.sck, board.spi1.miso, board.spi1.mosi),
            stm32f4xx_hal::spi::Mode {
                polarity: stm32f4xx_hal::spi::Polarity::IdleHigh,
                phase: stm32f4xx_hal::spi::Phase::CaptureOnSecondTransition,
            },
            1.MHz(),
            &board.clocks,
        );

        let mut cs = board.spi1.cs.into_push_pull_output();
        cs.set_high();
        let mut icm42688p = crate::icm42688p::Icm42688p::new(spi1, cs.into());
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
        // Configure timers for PWM output

        // This is the configuration in INAV:
        //     DEF_TIM(TIM4,   CH2, PB7,  TIM_USE_OUTPUT_AUTO,   1, 0), // S1 D(1,3,2)
        //     DEF_TIM(TIM4,   CH1, PB6,  TIM_USE_OUTPUT_AUTO,   1, 0), // S2 D(1,0,2)
        //
        //     DEF_TIM(TIM3,   CH3, PB0,  TIM_USE_OUTPUT_AUTO,   1, 0), // S3 D(1,7,5)
        //     DEF_TIM(TIM3,   CH4, PB1,  TIM_USE_OUTPUT_AUTO,   1, 0), // S4 D(1,2,5)
        //     DEF_TIM(TIM8,   CH3, PC8,  TIM_USE_OUTPUT_AUTO,   1, 0), // S5 D(2,4,7)
        //     DEF_TIM(TIM8,   CH4, PC9,  TIM_USE_OUTPUT_AUTO,   1, 0), // S6 D(2,7,7)

        // -----------------------------------------------------------------------------------------
        // Serial ELRS receiver

        let mut crsf_serial = board
            .USART1
            .serial(
                (board.usart1.tx, board.usart1.rx),
                stm32f4xx_hal::serial::Config::default().baudrate(420_000.bps()),
                &board.clocks,
            )
            .unwrap();
        crsf_serial.listen(stm32f4xx_hal::serial::Event::RxNotEmpty);

        // -----------------------------------------------------------------------------------------
        // Configure USB as CDC-ACM
        let mut usb_dp = board.usb_pins.dp.into_push_pull_output();
        usb_dp.set_low();
        cortex_m::asm::delay(board.clocks.sysclk().raw() / 100);

        let usb = stm32f4xx_hal::otg_fs::USB {
            usb_global: board.OTG_FS_GLOBAL,
            usb_device: board.OTG_FS_DEVICE,
            usb_pwrclk: board.OTG_FS_PWRCLK,
            pin_dm: board.usb_pins.dm.into(),
            pin_dp: usb_dp.into(),
            hclk: board.clocks.hclk(),
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

        // Spawn tasks
        imu_handler::spawn(imu_data_receiver, ahrs_state_sender).ok();
        crsf_parser::spawn(crsf_data_receiver, rc_state_sender).ok();
        control_task::spawn(ahrs_state_receiver, rc_state_receiver, pwm_output_sender).ok();
        pwm_output_task::spawn(pwm_output_receiver).ok();

        Mono::start(delay.release().release(), 168_000_000);
        (
            Shared {
                usb_dev,
                serial,
                flags: Flags {
                    reset_ahrs: AtomicBool::new(false),
                },
            },
            Local {
                dwt,
                icm42688p_dma_context,
                prev_fifo_count: 0,
                imu_data_sender,
                crsf_serial,
                crsf_data_sender,
                pwm_outputs: board.pwm_outputs,
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
            if true {
                cortex_m::asm::wfi();
            } else {
                cortex_m::asm::nop();
                counter += 1;
                if counter % 100_000 == 0 {
                    // defmt::info!("write tons of data in idle loop to check if this locks the FC");
                    // this makes the chip hang after 4.25 seconds, when unplugging the debugger while active
                    // it's fine when the chip is started without debugger
                }
                if counter >= 16_800_000 {
                    let cyc_cnt = dwt.cyccnt.read();
                    let time_passed = cyc_cnt.wrapping_sub(prev_cyc_cnt);
                    let idle_time = 16_800_000_f32 * 10.;
                    let ratio = 100. * (idle_time / time_passed as f32);

                    defmt::info!("Idle: {}", ratio);
                    prev_cyc_cnt = cyc_cnt;
                    counter = 0;
                }
            }
        }
    }

    extern "Rust" {
        #[task(priority = 10, local = [pwm_outputs])]
        async fn pwm_output_task(
            cx: pwm_output_task::Context,
            mut pwm_output_receiver: rtic_sync::channel::Receiver<'static, crate::OutputCommand, 1>,
        );

        #[task(
            binds = DMA2_STREAM0,
            shared = [],
            local = [icm42688p_dma_context, prev_fifo_count, imu_data_sender],
            priority = 5
        )]
        fn imu_rx_irq(cx: imu_rx_irq::Context);

        #[task(
            binds = USART1,
            shared = [],
            local = [crsf_serial, crsf_data_sender],
            priority = 5,
        )]
        fn usart1_irq(cx: usart1_irq::Context);

        #[task(priority = 3, shared = [&flags])]
        async fn crsf_parser(
            cx: crsf_parser::Context,
            mut rx: rtic_sync::channel::Receiver<'static, u8, 64>,
            mut tx: rtic_sync::channel::Sender<'static, RcState, 1>,
        );

        #[task(priority = 2, shared = [&flags])]
        async fn control_task(
            _cx: control_task::Context,
            mut ahrs_state_receiver: rtic_sync::channel::Receiver<'static, AhrsState, 1>,
            mut rc_state_receiver: rtic_sync::channel::Receiver<'static, RcState, 1>,
            mut pwm_output_sender: rtic_sync::channel::Sender<'static, crate::OutputCommand, 1>,
        );

        #[task(priority = 2, shared = [&flags])]
        async fn imu_handler(
            _cx: imu_handler::Context,
            mut imu_data_receiver: rtic_sync::channel::Receiver<'static, Box<IMUDATAPOOL>, 1>,
            mut ahrs_state_sender: rtic_sync::channel::Sender<'static, AhrsState, 1>,
        );

        #[task(binds = OTG_FS, shared = [usb_dev, serial], priority = 1)]
        fn usb_tx(cx: usb_tx::Context);

        #[task(priority = 1)]
        async fn blink(cx: blink::Context);

    }
}
