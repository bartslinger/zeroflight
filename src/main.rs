#![no_main]
#![no_std]
// #![deny(warnings)]
// #![deny(unsafe_code)]
//#![deny(missing_docs)]

use embassy_stm32::{gpio, spi, Config};
use embassy_time::Timer;

use embassy_executor::{Executor, InterruptExecutor, Spawner};
use embassy_stm32::gpio::OutputType;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::TIM8;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::{CountingMode, OutputCompareMode};
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_stm32::timer::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

// struct PwmOutputs {
//     s1: SimplePwmChannel<'static, TIM4>,
//     s2: SimplePwmChannel<'static, TIM4>,
// }

static EXECUTOR_MED: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

static WHO_AM_I_RESPONSE_CHANNEL: Watch<CriticalSectionRawMutex, u8, 64> = Watch::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    defmt::info!("Hello, world");

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL168,
            divp: Some(PllPDiv::DIV2), // 8mhz / 4 * 168 / 2 = 168Mhz.
            divq: Some(PllQDiv::DIV7), // 8mhz / 4 * 168 / 7 = 48Mhz.
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.mux.clk48sel = mux::Clk48sel::PLL1_Q;
    }
    let p = embassy_stm32::init(config);
    defmt::info!("initializing");

    let cs = gpio::Output::new(p.PA4, gpio::Level::High, gpio::Speed::VeryHigh);

    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(300_000);
    spi_config.mode = spi::MODE_3;
    let spi = embassy_stm32::spi::Spi::new(
        p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA2_CH3, p.DMA2_CH0, spi_config,
    );

    // Initialize PWM
    let _pwm_pin = PwmPin::<TIM8, _>::new_ch3(p.PC8, OutputType::PushPull);
    let timer = embassy_stm32::timer::low_level::Timer::new(p.TIM8);
    timer.set_counting_mode(CountingMode::EdgeAlignedUp);

    // set frequency part
    {
        let timer_f = timer.get_clock_frequency();
        // this will be 1_680_000:
        let pclk_ticks_per_timer_period = (timer_f / Hertz(50)) as u64;
        // If we **choose ourselves** to set psc to 83:
        // 1_680_000 / (83 + 1) = 20_000
        let psc: u16 = ((pclk_ticks_per_timer_period - 1) / 20_000) as u16;
        // this will be 20_000:
        let divide_by = pclk_ticks_per_timer_period / (u64::from(psc) + 1);
        // assume 16-bits
        let arr = defmt::unwrap!(u16::try_from(divide_by - 1));
        let regs = timer.regs_core();
        regs.psc().write_value(psc);
        regs.arr().write(|r| r.set_arr(arr));

        // I don't think this is really necessary:
        regs.cr1()
            .modify(|r| r.set_urs(embassy_stm32::pac::timer::vals::Urs::COUNTER_ONLY));
        regs.egr().write(|r| r.set_ug(true));
        regs.cr1()
            .modify(|r| r.set_urs(embassy_stm32::pac::timer::vals::Urs::ANY_EVENT));
    }

    timer.enable_outputs();
    timer.start();
    timer.set_output_compare_mode(Channel::Ch3, OutputCompareMode::PwmMode1);
    timer.set_output_compare_preload(Channel::Ch3, true);

    // enable ch3
    timer.set_compare_value(Channel::Ch3, 2000);
    timer.enable_channel(Channel::Ch3, true);

    let max_duty = timer.get_max_compare_value();
    defmt::info!("clock freq: {}", max_duty);

    // let pwm_tim4 = SimplePwm::new(
    //     tim4,
    //     Some(PwmPin::new_ch1(p.PB6, OutputType::PushPull)),
    //     Some(PwmPin::new_ch2(p.PB7, OutputType::PushPull)),
    //     None,
    //     None,
    //     Hertz(168),
    //     CountingMode::default(),
    // );
    // let max_duty = pwm_tim4.get_max_duty();
    // let pwm_tim4 = pwm_tim4.split();
    //
    // let pwm_outputs = PwmOutputs {
    //     s1: pwm_tim4.ch2,
    //     s2: pwm_tim4.ch1,
    // };

    // whyyyy
    interrupt::OTG_HS_EP1_OUT.set_priority(Priority::P1);
    let spawner = EXECUTOR_MED.start(interrupt::OTG_HS_EP1_OUT);
    if let Err(e) = spawner.spawn(run_med()) {
        defmt::error!("Failed to spawn med task: {}", e as u32);
    }

    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(run_low(spi, cs)).unwrap();
        spawner.spawn(idle()).unwrap();
    });
}

#[interrupt]
unsafe fn OTG_HS_EP1_OUT() {
    EXECUTOR_MED.on_interrupt();
}

#[embassy_executor::task]
async fn idle() {
    loop {
        // Prevent from going to sleep, otherwise SWD doesn't work anymore
        embassy_futures::yield_now().await;
    }
}

#[embassy_executor::task]
async fn run_med(/*pwm_outputs: PwmOutputs*/) {
    let mut receiver = WHO_AM_I_RESPONSE_CHANNEL.receiver().unwrap();
    loop {
        defmt::info!("    [med] waiting for byte...");
        let incoming_byte = receiver.changed().await;
        let dmaprio = interrupt::DMA2_STREAM0.get_priority();
        defmt::info!(
            "    [med] Received byte: {:02x}! (prio {})",
            incoming_byte,
            dmaprio
        );
        Timer::after_micros(50).await;
        // defmt::info!("    [med] block...");
        // embassy_time::block_for(embassy_time::Duration::from_secs(2)); // ~2 seconds
    }
}

#[embassy_executor::task]
async fn run_low(mut spi: spi::Spi<'static, Async>, mut cs: gpio::Output<'static>) {
    let sender = WHO_AM_I_RESPONSE_CHANNEL.sender();

    let mut count: u8 = 0;
    loop {
        // Do a SPI whoami
        let reg = 0x75;
        let mut buf: [u8; 2] = [reg | 0x80, 0x00];
        defmt::info!("[low] transfer start");
        cs.set_low();
        spi.transfer_in_place(&mut buf).await.unwrap();
        cs.set_high();
        defmt::info!("[low] response: {:02x}", buf[1]);
        sender.send(count);
        defmt::info!("[low] byte was sent");
        count += 1;

        Timer::after_millis(1000).await;
    }
}
// ---------------------------------------------- RTIC below ---------------------------------------
// Declare a pool
// box_pool!(IMUDATAPOOL: ImuData);

// mod pac {
//     pub use embassy_stm32::pac::Interrupt as interrupt;
//     pub use embassy_stm32::pac::*;
// }

// #[rtic::app(device = pac, peripherals = false, dispatchers = [OTG_HS_EP1_OUT, OTG_HS_EP1_IN, OTG_HS_WKUP, OTG_HS])]
// mod app {
//     use super::*;
//     use embassy_time::Timer;
// use crate::boards::board::{self, Board};
// use crate::common::{ActuatorPwmCommands, AhrsState, ImuData, RcState};
// use crate::drivers::icm42688p::Icm42688pDmaContext;
// use crate::hw_tasks::interrupt_handlers::dma2_stream0_irq;
// use crate::hw_tasks::interrupt_handlers::otg_fs_irq;
// use crate::hw_tasks::interrupt_handlers::usart1_irq;
// use crate::sw_tasks::ahrs::ahrs_task;
// use crate::sw_tasks::control::control_task;
// use crate::sw_tasks::crsf::crsf_parser_task;
// use crate::sw_tasks::pwm_output::pwm_output_task;
// use crate::IMUDATAPOOL;
// use heapless::pool::boxed::{Box, BoxBlock};
// use rtic_monotonics::systick::prelude::*;

// systick_monotonic!(Mono, 1_000);

// #[shared]
// struct Shared {
//     // usb_dev: usb_device::device::UsbDevice<'static, stm32f4xx_hal::otg_fs::UsbBusType>,
//     // serial: usbd_serial::SerialPort<'static, stm32f4xx_hal::otg_fs::UsbBusType>,
// }
//
// #[local]
// struct Local {
//     // dwt: cortex_m::peripheral::DWT,
//     // icm42688p_dma_context:
//     //     Icm42688pDmaContext<board::ImuDmaTxTransfer, board::ImuDmaRxTransfer, board::ImuCsPin>,
//     // prev_fifo_count: u16,
//     // imu_data_sender: rtic_sync::channel::Sender<'static, Box<IMUDATAPOOL>, 1>,
//     // crsf_serial: board::CrsfSerial,
//     // crsf_data_sender: rtic_sync::channel::Sender<'static, u8, 64>,
//     // pwm_outputs: board::PwmOutputs,
// }

// #[init(local = [
//     EP_MEMORY: [u32; 1024] = [0; 1024],
//     USB_BUS: Option<usb_device::bus::UsbBusAllocator<stm32f4xx_hal::otg_fs::UsbBusType>> =
//         None,
//     TX_BUFFER_1: [u8; 129] = [0x00; 129],
//     TX_BUFFER_2: [u8; 129] = [0x00; 129],
//     RX_BUFFER_1: [u8; 129] = [0x00; 129],
//     RX_BUFFER_2: [u8; 129] = [0x00; 129],
//     IMU_DATA_CHANNEL_MEMORY: [BoxBlock<crate::ImuData>; 2] = [const { BoxBlock::new() }; 2],
// ])]
// #[init]
// fn init(cx: init::Context) -> (Shared, Local) {
//     let p = embassy_stm32::init(Default::default());
//
//     // Mono::start(cx.core.SYST, 168_000_000);
//
//     defmt::info!("Hello, world");
//     embassy_test::spawn().ok();
// let board = Board::from(cx.device);
//
// defmt::info!("init");
//
// for block in cx.local.IMU_DATA_CHANNEL_MEMORY {
//     IMUDATAPOOL.manage(block);
// }
//
// let (imu_data_sender, imu_data_receiver) = rtic_sync::make_channel!(Box<IMUDATAPOOL>, 1);
// let (imu_to_ahrs_data_sender, imu_to_ahrs_data_receiver) =
//     rtic_sync::make_channel!(ImuData, 1);
// let (ahrs_state_tx, ahrs_state_rx) = rtic_sync::make_channel!(AhrsState, 1);
// let (crsf_data_sender, crsf_data_receiver) = rtic_sync::make_channel!(u8, 64);
// let (rc_state_sender, rc_state_receiver) = rtic_sync::make_channel!(RcState, 1);
// let (pwm_output_sender, pwm_output_receiver) =
//     rtic_sync::make_channel!(ActuatorPwmCommands, 1);
//
// let mut delay = cx.core.SYST.delay(&board.clocks);
//
// // -----------------------------------------------------------------------------------------
// // Enable cycle counter for counting clock ticks
// let mut dwt = cx.core.DWT;
// dwt.enable_cycle_counter();
// let mut dcb = cx.core.DCB;
// dcb.enable_trace();
//
// // -----------------------------------------------------------------------------------------
// // DMA
// let dma2 = stm32f4xx_hal::dma::StreamsTuple::new(board.DMA2);
//
// // -----------------------------------------------------------------------------------------
// // Scan I2C
// defmt::info!("Start I2C scan...");
//
// let mut i2c1 = stm32f4xx_hal::i2c::I2c::new(
//     board.I2C1,
//     (board.i2c1.scl, board.i2c1.sda),
//     stm32f4xx_hal::i2c::Mode::from(400.kHz()),
//     &board.clocks,
// );
//
// const VALID_ADDR_RANGE: core::ops::Range<u8> = 0x08..0x78;
//
// for addr in 0x00_u8..0x80 {
//     let byte: [u8; 1] = [0; 1];
//     if VALID_ADDR_RANGE.contains(&addr) && i2c1.write(addr, &byte).is_ok() {
//         defmt::info!("Found device at {:02x}", addr);
//     }
// }
//
// // -----------------------------------------------------------------------------------------
// // Initialize SPL06
// // Not complete
// let start = dwt.cyccnt.read();
//
// let mut buf = [0];
// let res = i2c1.write_read(0x76_u8, &[0x0D], &mut buf);
// if let Err(_) = res {
//     defmt::error!("I2C write_read error");
// }
//
// let end = dwt.cyccnt.read();
// let diff = end.wrapping_sub(start);
// defmt::info!("Baro initialization cycle count: {}", diff);
//
// defmt::info!("WHO_AM_I: {:02x}", buf[0]);
//
// // -----------------------------------------------------------------------------------------
// // Configure ICM-42688-P on SPI
//
// let spi1 = stm32f4xx_hal::spi::Spi::new(
//     board.SPI1,
//     (board.spi1.sck, board.spi1.miso, board.spi1.mosi),
//     stm32f4xx_hal::spi::Mode {
//         polarity: stm32f4xx_hal::spi::Polarity::IdleHigh,
//         phase: stm32f4xx_hal::spi::Phase::CaptureOnSecondTransition,
//     },
//     1.MHz(),
//     &board.clocks,
// );
//
// let mut cs = board.spi1.cs.into_push_pull_output();
// cs.set_high();
// let mut icm42688p = crate::drivers::icm42688p::Icm42688p::new(spi1, cs);
// icm42688p.init(&mut delay);
// let (spi1, cs) = icm42688p.release();
//
// let icm42688p_dma_context = crate::drivers::icm42688p::start_dma(
//     cx.local.TX_BUFFER_1,
//     cx.local.RX_BUFFER_1,
//     cx.local.TX_BUFFER_2,
//     cx.local.RX_BUFFER_2,
//     spi1,
//     dma2.3,
//     dma2.0,
//     cs,
// );
//
// // -----------------------------------------------------------------------------------------
// // Serial ELRS receiver
//
// let mut crsf_serial = board
//     .USART1
//     .serial(
//         (board.usart1.tx, board.usart1.rx),
//         stm32f4xx_hal::serial::Config::default().baudrate(420_000.bps()),
//         &board.clocks,
//     )
//     .unwrap();
// crsf_serial.listen(stm32f4xx_hal::serial::Event::RxNotEmpty);
//
// // -----------------------------------------------------------------------------------------
// // Configure USB as CDC-ACM
// let mut usb_dp = board.usb_pins.dp.into_push_pull_output();
// usb_dp.set_low();
// cortex_m::asm::delay(board.clocks.sysclk().raw() / 100);
//
// let usb = stm32f4xx_hal::otg_fs::USB {
//     usb_global: board.OTG_FS_GLOBAL,
//     usb_device: board.OTG_FS_DEVICE,
//     usb_pwrclk: board.OTG_FS_PWRCLK,
//     pin_dm: board.usb_pins.dm.into(),
//     pin_dp: usb_dp.into(),
//     hclk: board.clocks.hclk(),
// };
//
// let usb_bus = stm32f4xx_hal::otg_fs::UsbBus::new(usb, cx.local.EP_MEMORY);
// *cx.local.USB_BUS = Some(usb_bus);
//
// let usb_bus_reference = cx.local.USB_BUS.as_ref().unwrap();
// let serial = usbd_serial::SerialPort::new(usb_bus_reference);
// let usb_dev = usb_device::device::UsbDeviceBuilder::new(
//     usb_bus_reference,
//     usb_device::device::UsbVidPid(0x16c0, 0x27dd),
// )
// .device_class(usbd_serial::USB_CLASS_CDC)
// .strings(&[usb_device::device::StringDescriptors::default()
//     .manufacturer("ZeroFlight")
//     .product("SpeedyBee F405 Wing")
//     .serial_number("TEST")])
// .unwrap()
// .build();
//
// // Spawn software tasks
// crsf_parser_task::spawn(crsf_data_receiver, rc_state_sender).ok();
// control_task::spawn(
//     imu_data_receiver,
//     imu_to_ahrs_data_sender,
//     ahrs_state_rx,
//     rc_state_receiver,
//     pwm_output_sender,
// )
// .ok();
// ahrs_task::spawn(imu_to_ahrs_data_receiver, ahrs_state_tx).ok();
// pwm_output_task::spawn(pwm_output_receiver).ok();
//
// Mono::start(delay.release().release(), 168_000_000);
// (
//     Shared { usb_dev, serial },
//     Local {
//         dwt,
//         icm42688p_dma_context,
//         prev_fifo_count: 0,
//         imu_data_sender,
//         crsf_serial,
//         crsf_data_sender,
//         pwm_outputs: board.pwm_outputs,
//     },
// )
// (Shared {}, Local {})
// }

// #[idle]
// fn idle(_cx: idle::Context) -> ! {
//     // let dwt = cx.local.dwt;
//     //
//     // let mut prev_cyc_cnt = dwt.cyccnt.read();
//     // let mut counter = 0;
//     loop {
//         if true {
//             // cortex_m::asm::wfi();
//             cortex_m::asm::nop();
//         } else {
//             // cortex_m::asm::nop();
//             // counter += 1;
//             // if counter % 100_000 == 0 {
//             //     // defmt::info!("write tons of data in idle loop to check if this locks the FC");
//             //     // this makes the chip hang after 4.25 seconds, when unplugging the debugger while active
//             //     // it's fine when the chip is started without debugger
//             // }
//             // if counter >= 16_800_000 {
//             //     let cyc_cnt = dwt.cyccnt.read();
//             //     let time_passed = cyc_cnt.wrapping_sub(prev_cyc_cnt);
//             //     let idle_time = 16_800_000_f32 * 10.;
//             //     let ratio = 100. * (idle_time / time_passed as f32);
//             //
//             //     defmt::info!("Idle: {}", ratio);
//             //     prev_cyc_cnt = cyc_cnt;
//             //     counter = 0;
//             // }
//         }
//     }
// }
//
// #[task(priority = 1)]
// async fn embassy_test(_cx: embassy_test::Context) {
//     loop {
//         defmt::info!("loop");
//         Timer::after_millis(1000).await;
//         // Mono::delay(1_000.millis()).await;
//     }
// }

// extern "Rust" {
//     #[task(priority = 10, local = [pwm_outputs])]
//     async fn pwm_output_task(
//         cx: pwm_output_task::Context,
//         mut pwm_output_receiver: rtic_sync::channel::Receiver<'static, ActuatorPwmCommands, 1>,
//     );
//
//     #[task(
//         binds = DMA2_STREAM0,
//         shared = [],
//         local = [icm42688p_dma_context, prev_fifo_count, imu_data_sender],
//         priority = 5
//     )]
//     fn dma2_stream0_irq(cx: dma2_stream0_irq::Context);
//
//     #[task(
//         binds = USART1,
//         shared = [],
//         local = [crsf_serial, crsf_data_sender],
//         priority = 5,
//     )]
//     fn usart1_irq(cx: usart1_irq::Context);
//
//     #[task(priority = 4, shared = [])]
//     async fn crsf_parser_task(
//         cx: crsf_parser_task::Context,
//         mut rx: rtic_sync::channel::Receiver<'static, u8, 64>,
//         mut tx: rtic_sync::channel::Sender<'static, RcState, 1>,
//     );
//
//     #[task(priority = 3, local = [dwt], shared = [])]
//     async fn control_task(
//         _cx: control_task::Context,
//         mut imu_data_receiver: rtic_sync::channel::Receiver<'static, Box<IMUDATAPOOL>, 1>,
//         mut imu_to_ahrs_data_sender: rtic_sync::channel::Sender<'static, ImuData, 1>,
//         mut ahrs_state_rx: rtic_sync::channel::Receiver<'static, AhrsState, 1>,
//         mut rc_state_receiver: rtic_sync::channel::Receiver<'static, RcState, 1>,
//         mut pwm_output_sender: rtic_sync::channel::Sender<'static, ActuatorPwmCommands, 1>,
//     );
//
//     #[task(priority = 2)]
//     async fn ahrs_task(
//         cx: ahrs_task::Context,
//         mut imu_to_ahrs_data_receiver: rtic_sync::channel::Receiver<'static, ImuData, 1>,
//         mut ahrs_state_tx: rtic_sync::channel::Sender<'static, AhrsState, 1>,
//     );
//
//     #[task(binds = OTG_FS, shared = [usb_dev, serial], priority = 1)]
//     fn otg_fs_irq(cx: otg_fs_irq::Context);
// }
// }
