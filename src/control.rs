use crate::controller::ControllerOutput;
use crate::crsf::RcState;
use crate::imu::AhrsState;
use core::sync::atomic::Ordering::SeqCst;

const PI: f32 = 3.14159265358979323846264338327950288_f32;

pub(crate) async fn control_task(
    cx: crate::app::control_task::Context<'_>,
    mut ahrs_state_receiver: rtic_sync::channel::Receiver<'static, AhrsState, 1>,
    mut rc_state_receiver: rtic_sync::channel::Receiver<'static, RcState, 1>,
    mut pwm_output_sender: rtic_sync::channel::Sender<'static, crate::OutputCommand, 1>,
) {
    use crate::app::Mono;
    use futures::{select_biased, FutureExt};
    use rtic_monotonics::Monotonic;

    enum FlightMode {
        Stabilized,
        Acro,
        Manual,
        Failsafe,
    }
    let mut flight_mode = FlightMode::Stabilized;

    let mut rc_timestamp = Mono::now();
    let mut rc_state = RcState {
        armed: false,
        roll: 1500,
        pitch: 1500,
        throttle: 900,
        yaw: 1500,
        mode: 1000,
        pitch_offset: 1000,
    };

    let mut ahrs_state;

    let mut controller = crate::controller::Controller::new();

    defmt::info!("control task started");
    loop {
        enum ControlTaskEvent {
            RcState(RcState),
            AhrsState(AhrsState),
        }

        let event = select_biased! {
            v = rc_state_receiver.recv().fuse() => {
                match v {
                    Ok(v) => ControlTaskEvent::RcState(v),
                    Err(_) => continue,
                }
            }
            v = ahrs_state_receiver.recv().fuse() => {
                match v {
                    Ok(v) => ControlTaskEvent::AhrsState(v),
                    Err(_) => continue,
                }
            }
        };

        match event {
            ControlTaskEvent::RcState(new_rc_state) => {
                rc_state = new_rc_state;
                rc_timestamp = Mono::now();
                if new_rc_state.mode < 1400 {
                    flight_mode = FlightMode::Stabilized;
                    // stabilized mode will be active on next ahrs sample
                } else if new_rc_state.mode >= 1400 && new_rc_state.mode < 1600 {
                    flight_mode = FlightMode::Acro;
                } else {
                    // manual mode
                    flight_mode = FlightMode::Manual;
                    let output_command = crate::OutputCommand {
                        armed: rc_state.armed,
                        roll: rc_state.roll,
                        pitch: rc_state.pitch,
                        throttle: rc_state.throttle,
                        yaw: rc_state.yaw,
                    };
                    pwm_output_sender.try_send(output_command).ok();
                }
                // This is important, we don't process the rc commands directly in stabilized modes.
                // IMU data is leading
                // It's a design choice that could be set differently later,
                // For example, the previous PID calculation could be re-calculated based on new input
                continue;
            }
            ControlTaskEvent::AhrsState(new_ahrs_state) => {
                ahrs_state = new_ahrs_state;
                if let Some(dt) = Mono::now().checked_duration_since(rc_timestamp) {
                    if rc_state.armed && dt.to_millis() > 500 {
                        flight_mode = FlightMode::Failsafe;
                    }
                }
            }
        }

        if cx.shared.flags.reset_ahrs.load(SeqCst) {
            controller.reset();
        }
        match flight_mode {
            FlightMode::Manual => {}
            FlightMode::Stabilized => {
                let pitch_offset = if rc_state.pitch_offset > 1500 {
                    -(rc_state.pitch_offset as i16 - 1500)
                } else {
                    0
                };
                let roll_setpoint =
                    ((rc_state.roll as i16 - 1500) as f32 / 500.0) * 45.0 * PI / 180.0;
                let pitch_setpoint =
                    ((rc_state.pitch as i16 + pitch_offset - 1500) as f32 / 500.0) * -30.0 * PI
                        / 180.0;
                let pitch_offset = 2.0 * PI / 180.0; // 2 degrees pitch up by default
                let ControllerOutput { roll, pitch } = controller.update(
                    ahrs_state,
                    roll_setpoint,
                    pitch_offset + pitch_setpoint,
                    rc_state.armed,
                );
                let output_command = crate::OutputCommand {
                    armed: rc_state.armed,
                    roll: ((roll * 500.0) as i16 + 1500) as u16,
                    pitch: ((pitch * 500.0) as i16 + 1500) as u16,
                    throttle: rc_state.throttle,
                    yaw: rc_state.yaw,
                };
                pwm_output_sender.try_send(output_command).ok();
            }
            FlightMode::Acro => {
                // 180deg/s roll, 90 deg/s pitch
                let roll_rate_setpoint =
                    ((rc_state.roll as i16 - 1500) as f32 / 500.0) * 180.0 * PI / 180.0;
                let pitch_rate_setpoint =
                    ((rc_state.pitch as i16 - 1500) as f32 / 500.0) * -90.0 * PI / 180.0;

                let ControllerOutput { roll, pitch } = controller.stabilize_rates(
                    ahrs_state,
                    roll_rate_setpoint,
                    pitch_rate_setpoint,
                    rc_state.armed,
                );
                let output_command = crate::OutputCommand {
                    armed: rc_state.armed,
                    roll: ((roll * 500.0) as i16 + 1500) as u16,
                    pitch: ((pitch * 500.0) as i16 + 1500) as u16,
                    throttle: rc_state.throttle,
                    yaw: rc_state.yaw,
                };
                pwm_output_sender.try_send(output_command).ok();
            }
            FlightMode::Failsafe => {
                let ControllerOutput { roll, pitch } =
                    controller.update(ahrs_state, 0.0, 0.0, true);
                let output_command = crate::OutputCommand {
                    armed: false,
                    roll: ((roll * 500.0) as i16 + 1500) as u16,
                    pitch: ((pitch * 500.0) as i16 + 1500) as u16,
                    throttle: 900,
                    yaw: 1500,
                };
                pwm_output_sender.try_send(output_command).ok();
            }
        }
    }
}
