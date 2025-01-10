use crate::common::{ActuatorPwmCommands, ImuData, RcState, Value, PI};
use crate::vehicle::modes::Mode;
use crate::vehicle::{main_loop, MainState};
use crate::IMUDATAPOOL;
use heapless::pool::boxed::Box;

pub(crate) async fn control_task(
    _cx: crate::app::control_task::Context<'_>,
    mut imu_data_receiver: rtic_sync::channel::Receiver<'static, Box<IMUDATAPOOL>, 1>,
    mut rc_state_receiver: rtic_sync::channel::Receiver<'static, RcState, 1>,
    mut pwm_output_sender: rtic_sync::channel::Sender<'static, ActuatorPwmCommands, 1>,
) {
    use futures::{select_biased, FutureExt};

    let mut mode = Mode::Stabilized;

    let mut rc_state = Value::new([0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);

    let mut imu_data = Value::new(ImuData {
        acceleration: (0.0, 0.0, 0.0),
        rates: (0.0, 0.0, 0.0),
    });

    let mut main_loop_state = MainState::default();

    defmt::info!("control task started");
    loop {
        // Main reason for using this enum is because select_biased! doesn't work with code
        // formatting
        enum ControlTaskEvent {
            ImuData(Box<IMUDATAPOOL>),
            RcState(RcState),
        }

        let event = select_biased! {
            v = imu_data_receiver.recv().fuse() => {
                match v {
                    Ok(v) => ControlTaskEvent::ImuData(v),
                    Err(_) => continue,
                }
            }
            v = rc_state_receiver.recv().fuse() => {
                match v {
                    Ok(v) => ControlTaskEvent::RcState(v),
                    Err(_) => continue,
                }
            }
        };

        match event {
            ControlTaskEvent::ImuData(raw_imu_data) => {
                let parsed_imu_data = parse_imu_data(raw_imu_data);
                imu_data.update(parsed_imu_data);
            }
            ControlTaskEvent::RcState(new_rc_state) => {
                rc_state.update(new_rc_state);
            } // TODO: Timeout? if no imu data, map rc directly to output?
        }

        if let Some(imu_value) = imu_data.updated() {
            let output = main_loop(&mut main_loop_state, &mut mode, imu_value, &mut rc_state);
            if let Err(_) = pwm_output_sender.try_send(output) {
                defmt::error!("error sending pwm output");
            }
        }
    }
}

fn parse_imu_data(buf: Box<IMUDATAPOOL>) -> ImuData {
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
    let _timestamp: u16 = (raw_timestamp as u32 * 32 / 30) as u16;

    ImuData {
        acceleration: (acc_x, acc_y, acc_z),
        rates: (gyro_x, gyro_y, gyro_z),
    }
}
