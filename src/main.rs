#![no_main]
#![no_std]
//#![deny(warnings)]
#![deny(unsafe_code)]
//#![deny(missing_docs)]

mod blink;
mod crsf;
mod icm42688p;
mod imu;
mod usb;

use defmt_rtt as _;
use panic_halt as _;

struct OutputCommand {
    armed: bool,
    roll: u16,
    pitch: u16,
    throttle: u16,
    yaw: u16,
}

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [OTG_HS_EP1_OUT, OTG_HS_EP1_IN, OTG_HS_WKUP, OTG_HS])]
mod app {
    use crate::blink::blink;
    use crate::crsf::RcState;
    use crate::imu::imu_rx_handler;
    use crate::usb::usb_tx;
    use rtic_monotonics::systick::prelude::*;
    use stm32f4xx_hal::{gpio, nb};

    const PI: f32 = 3.14159265358979323846264338327950288_f32;

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
        imu_data_sender: rtic_sync::channel::Sender<'static, [u8; 16], 1>,
        crsf_serial: stm32f4xx_hal::serial::Serial<stm32f4xx_hal::pac::USART1, u8>,
        crsf_data_sender: rtic_sync::channel::Sender<'static, u8, 64>,
        s1: stm32f4xx_hal::timer::PwmChannel<stm32f4xx_hal::pac::TIM4, 1>,
        s5: stm32f4xx_hal::timer::PwmChannel<stm32f4xx_hal::pac::TIM8, 2>,
        s6: stm32f4xx_hal::timer::PwmChannel<stm32f4xx_hal::pac::TIM8, 3>,
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

        let (imu_data_sender, imu_data_receiver) = rtic_sync::make_channel!([u8; 16], 1);
        let (crsf_data_sender, crsf_data_receiver) = rtic_sync::make_channel!(u8, 64);
        let (rc_state_sender, rc_state_receiver) = rtic_sync::make_channel!(RcState, 1);
        let (pwm_output_sender, pwm_output_receiver) =
            rtic_sync::make_channel!(crate::OutputCommand, 1);
        let (ahrs_state_sender, ahrs_state_receiver) =
            rtic_sync::make_channel!(dcmimu::EulerAngles, 1);

        imu_handler::spawn(imu_data_receiver, ahrs_state_sender).ok();
        crsf_parser::spawn(crsf_data_receiver, rc_state_sender).ok();
        control_task::spawn(ahrs_state_receiver, rc_state_receiver, pwm_output_sender).ok();
        pwm_output_task::spawn(pwm_output_receiver).ok();

        defmt::info!("Clock setup");
        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(168.MHz()).require_pll48clk().freeze();
        defmt::info!("SYSCLK: {}", clocks.sysclk().raw());
        defmt::info!("HCLK: {}", clocks.hclk().raw());
        defmt::info!("PCLK1: {}", clocks.pclk1().raw());
        defmt::info!("PCLK2: {}", clocks.pclk2().raw());
        assert!(clocks.is_pll48clk_valid());

        let mut delay = cx.core.SYST.delay(&clocks);

        let gpioa = cx.device.GPIOA.split();
        let gpiob = cx.device.GPIOB.split();
        let gpioc = cx.device.GPIOC.split();
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
        // Configure timers for PWM output

        // This is the configuration in INAV:
        //     DEF_TIM(TIM4,   CH2, PB7,  TIM_USE_OUTPUT_AUTO,   1, 0), // S1 D(1,3,2)
        //     DEF_TIM(TIM4,   CH1, PB6,  TIM_USE_OUTPUT_AUTO,   1, 0), // S2 D(1,0,2)
        //
        //     DEF_TIM(TIM3,   CH3, PB0,  TIM_USE_OUTPUT_AUTO,   1, 0), // S3 D(1,7,5)
        //     DEF_TIM(TIM3,   CH4, PB1,  TIM_USE_OUTPUT_AUTO,   1, 0), // S4 D(1,2,5)
        //     DEF_TIM(TIM8,   CH3, PC8,  TIM_USE_OUTPUT_AUTO,   1, 0), // S5 D(2,4,7)
        //     DEF_TIM(TIM8,   CH4, PC9,  TIM_USE_OUTPUT_AUTO,   1, 0), // S6 D(2,7,7)

        let (_, (_, _, tim3_ch3, tim3_ch4)) = cx.device.TIM3.pwm_us(20_000.micros(), &clocks);
        let (_, (tim4_ch1, tim4_ch2, _, _)) = cx.device.TIM4.pwm_us(20_000.micros(), &clocks);

        let (_, (_, _, tim8_ch3, tim8_ch4)) = cx.device.TIM8.pwm_us(20_000.micros(), &clocks);

        let mut s1 = tim4_ch2.with(gpiob.pb7);
        let mut s2 = tim4_ch1.with(gpiob.pb6);
        let mut s3 = tim3_ch3.with(gpiob.pb0);
        let mut s4 = tim3_ch4.with(gpiob.pb1);
        let mut s5 = tim8_ch3.with(gpioc.pc8);
        let mut s6 = tim8_ch4.with(gpioc.pc9);

        s1.set_duty(900);
        s1.enable();

        s2.set_duty(900);
        s2.enable();

        s3.set_duty(1500);
        s3.enable();

        s4.set_duty(1500);
        s4.enable();

        s5.set_duty(1500);
        s5.enable();

        s6.set_duty(1500);
        s6.enable();

        // -----------------------------------------------------------------------------------------
        // Serial ELRS receiver

        let usart1 = cx.device.USART1;
        let tx = gpioa.pa9;
        let rx = gpioa.pa10;

        let mut crsf_serial = usart1
            .serial(
                (tx, rx),
                stm32f4xx_hal::serial::Config::default().baudrate(420_000.bps()),
                &clocks,
            )
            .unwrap();
        crsf_serial.listen(stm32f4xx_hal::serial::Event::RxNotEmpty);

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
                crsf_serial,
                crsf_data_sender,
                s1,
                s5,
                s6,
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
        mut imu_data_receiver: rtic_sync::channel::Receiver<'static, [u8; 16], 1>,
        mut ahrs_state_sender: rtic_sync::channel::Sender<'static, dcmimu::EulerAngles, 1>,
    ) {
        defmt::info!("imu handler spawned");
        let mut ahrs = dcmimu::DCMIMU::new();
        let mut i = 0;
        let mut prev_timestamp = 0;
        while let Ok(buf) = imu_data_receiver.recv().await {
            i += 1;
            if i % 10 != 0 {
                // only do 100Hz, this imu estimator is slow
                continue;
            }
            let raw_acc_x = i16::from_be_bytes([buf[1], buf[2]]);
            let raw_acc_y = i16::from_be_bytes([buf[3], buf[4]]);
            let raw_acc_z = i16::from_be_bytes([buf[5], buf[6]]);
            let raw_gyro_x = i16::from_be_bytes([buf[7], buf[8]]);
            let raw_gyro_y = i16::from_be_bytes([buf[9], buf[10]]);
            let raw_gyro_z = i16::from_be_bytes([buf[11], buf[12]]);
            let raw_temperature = buf[13];
            let raw_timestamp = u16::from_be_bytes([buf[14], buf[15]]);
            let acc_x = raw_acc_x as f32 * 9.80665 / 2048.0;
            let acc_y = -raw_acc_y as f32 * 9.80665 / 2048.0;
            let acc_z = raw_acc_z as f32 * 9.80665 / 2048.0;
            let gyro_x = -raw_gyro_x as f32 * PI / 180.0 / 16.4;
            let gyro_y = raw_gyro_y as f32 * PI / 180.0 / 16.4;
            let gyro_z = -raw_gyro_z as f32 * PI / 180.0 / 16.4;
            let _temperature_celsius = (raw_temperature as f32 / 2.07) + 25.0;
            let timestamp: u16 = (raw_timestamp as u32 * 32 / 30) as u16;
            let diff = timestamp.wrapping_sub(prev_timestamp);
            prev_timestamp = timestamp;

            let (dcm, _gyro_bias) =
                ahrs.update((gyro_x, gyro_y, gyro_z), (acc_x, acc_y, acc_z), 0.01);

            if let Err(_) = ahrs_state_sender.try_send(dcm) {
                // defmt::error!("error publishing ahrs state");
            }
            if i % 1000 == 0 {
                // defmt::info!("gyro x: {}\ty: {}\tz: {}", gyro_x, gyro_y, gyro_z);
                // defmt::info!("accel x: {}\ty: {}\tz: {}", acc_x, acc_y, acc_z);
                defmt::info!(
                    "T: {}\troll: {}\tpitch: {}\tyaw: {}",
                    diff,
                    dcm.roll * 180.0 / PI,
                    dcm.pitch * 180.0 / PI,
                    dcm.yaw * 180.0 / PI
                );
                i = 0;
            }
        }
    }

    #[task(priority = 3)]
    async fn crsf_parser(
        _cx: crsf_parser::Context,
        mut rx: rtic_sync::channel::Receiver<'static, u8, 64>,
        mut tx: rtic_sync::channel::Sender<'static, RcState, 1>,
    ) {
        defmt::info!("starting crsf parser");
        let mut armed = false;
        let mut previous_armed_channel_state: u16 = 1000;
        loop {
            // Check sync byte
            if rx.recv().await.unwrap() != 0xC8 {
                continue;
            }
            let length = rx.recv().await.unwrap();
            if length != 24 {
                continue;
            }
            // Check message type RC Channels Packed Payload
            if rx.recv().await.unwrap() != 0x16 {
                continue;
            }
            let mut channel_bytes = [0_u8; 22];
            for i in 0..22 {
                channel_bytes[i] = rx.recv().await.unwrap();
            }
            let channels = crate::crsf::Channels::from_bytes(channel_bytes);
            let roll = crate::crsf::ticks_to_us(channels.channel_01());
            let pitch = crate::crsf::ticks_to_us(channels.channel_02());
            let throttle = crate::crsf::ticks_to_us(channels.channel_03());
            let yaw = crate::crsf::ticks_to_us(channels.channel_04());
            let armed_channel = crate::crsf::ticks_to_us(channels.channel_05());
            let mode = crate::crsf::ticks_to_us(channels.channel_06());

            if previous_armed_channel_state <= 1500 && armed_channel > 1500 && throttle <= 1000 {
                armed = true;
            }
            if armed_channel < 1500 {
                armed = false;
            }
            previous_armed_channel_state = armed_channel;

            let rc_in = RcState {
                armed,
                roll,
                pitch,
                throttle,
                yaw,
                mode,
            };
            if let Err(_) = tx.try_send(rc_in) {
                defmt::error!("error publishing rc state");
            }
        }
    }

    #[task(priority = 2)]
    async fn control_task(
        _cx: control_task::Context,
        mut ahrs_state_receiver: rtic_sync::channel::Receiver<'static, dcmimu::EulerAngles, 1>,
        mut rc_state_receiver: rtic_sync::channel::Receiver<'static, RcState, 1>,
        mut pwm_output_sender: rtic_sync::channel::Sender<'static, crate::OutputCommand, 1>,
    ) {
        use futures::{select_biased, FutureExt};
        defmt::info!("control task started");
        let mut saved_rc_state = RcState {
            armed: false,
            roll: 1500,
            pitch: 1500,
            throttle: 900,
            yaw: 1500,
            mode: 1000,
        };

        loop {
            enum ControlTaskEvent {
                RcState(RcState),
                AhrsState(dcmimu::EulerAngles),
            }

            let event = select_biased! {
                v = rc_state_receiver.recv().fuse() => {
                    let rc_state = v.unwrap();
                    ControlTaskEvent::RcState(rc_state)
                }
                v = ahrs_state_receiver.recv().fuse() => {
                    let ahrs_state = v.unwrap();
                    ControlTaskEvent::AhrsState(ahrs_state)
                }
            };

            let output_command = match event {
                ControlTaskEvent::RcState(rc_state) => {
                    saved_rc_state = rc_state;
                    if rc_state.mode < 1450 {
                        // save the command and use it in stabilized mode
                        None
                    } else {
                        // manual mode
                        let output_command = crate::OutputCommand {
                            armed: rc_state.armed,
                            roll: rc_state.roll,
                            pitch: rc_state.pitch,
                            throttle: rc_state.throttle,
                            yaw: rc_state.yaw,
                        };
                        Some(output_command)
                    }
                }
                ControlTaskEvent::AhrsState(ahrs_state) => {
                    if saved_rc_state.mode >= 1450 {
                        // we don't do anything with the ahrs data in manual mode
                        None
                    } else {
                        let pitch_commad = (saved_rc_state.pitch as i16
                            + (ahrs_state.pitch * 180.0 / PI * 20.0) as i16)
                            as u16;
                        let output_command = crate::OutputCommand {
                            armed: saved_rc_state.armed,
                            roll: saved_rc_state.roll,
                            pitch: pitch_commad,
                            throttle: saved_rc_state.throttle,
                            yaw: saved_rc_state.yaw,
                        };
                        Some(output_command)
                    }
                }
            };
            if let Some(v) = output_command {
                pwm_output_sender.try_send(v).ok();
            }
        }
    }

    #[task(priority = 10, local = [s1, s5, s6])]
    async fn pwm_output_task(
        cx: pwm_output_task::Context,
        mut pwm_output_receiver: rtic_sync::channel::Receiver<'static, crate::OutputCommand, 1>,
    ) {
        defmt::info!("pwm output task started");
        loop {
            match Mono::timeout_after(100.millis(), pwm_output_receiver.recv()).await {
                Ok(v) => {
                    let rc_state = v.unwrap();
                    // defmt::info!(
                    //     "channel info package ready {} {} {} {} {}",
                    //     rc_state.armed,
                    //     rc_state.roll,
                    //     rc_state.pitch,
                    //     rc_state.throttle,
                    //     rc_state.yaw,
                    // );
                    if rc_state.armed {
                        let throttle = rc_state.throttle.min(2000).max(1000);
                        cx.local.s1.set_duty(throttle);
                    } else {
                        cx.local.s1.set_duty(900);
                    };
                    let s5_duty = ((rc_state.pitch as i16 - 1500) * -1 + 1500) as u16;
                    let s6_duty = rc_state.pitch;

                    cx.local.s5.set_duty(s5_duty.min(2000).max(1000));
                    cx.local.s6.set_duty(s6_duty.min(2000).max(1000));
                }
                Err(_) => {
                    defmt::error!("pwm output timeout");
                    cx.local.s1.set_duty(900);
                    cx.local.s5.set_duty(1500);
                    cx.local.s6.set_duty(1500);
                }
            }
        }
    }

    #[task(
        binds = USART1,
        shared = [],
        local = [crsf_serial, crsf_data_sender],
        priority = 5,
    )]
    fn usart1_irq(cx: usart1_irq::Context) {
        use stm32f4xx_hal::prelude::*;
        let serial = cx.local.crsf_serial;
        if serial.is_rx_not_empty() {
            let byte = match serial.read() {
                Ok(v) => v,
                Err(nb::Error::Other(e)) => {
                    defmt::info!("Error in receiving usart1 byte: {}", e as u32);
                    return;
                }
                Err(nb::Error::WouldBlock) => {
                    defmt::info!("Error in receiving usart1 byte: would block");
                    return;
                }
            };
            if let Err(_e) = cx.local.crsf_data_sender.try_send(byte) {
                defmt::error!("send error from usart1 interrupt");
            };
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
