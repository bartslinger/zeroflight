use crate::behavior::controller::{Controller, ControllerOutput};
use crate::drivers::crsf::RcState;
use crate::sw_tasks::imu::AhrsState;
use crate::OutputCommand;
use core::sync::atomic::Ordering::SeqCst;

use crate::common::PI;

enum FlightMode {
    Stabilized,
    Acro,
    Manual,
    Failsafe,
}

pub(crate) async fn control_task(
    cx: crate::app::control_task::Context<'_>,
    mut ahrs_state_receiver: rtic_sync::channel::Receiver<'static, AhrsState, 1>,
    mut rc_state_receiver: rtic_sync::channel::Receiver<'static, RcState, 1>,
    mut pwm_output_sender: rtic_sync::channel::Sender<'static, crate::OutputCommand, 1>,
) {
    use crate::app::Mono;
    use futures::{select_biased, FutureExt};
    use rtic_monotonics::Monotonic;

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

    let mut ahrs_state = AhrsState {
        angles: dcmimu::EulerAngles::default(),
        rates: (0.0, 0.0, 0.0),
        _acceleration: (0.0, 0.0, 0.0),
    };

    let controller = Controller::new();
    let mut previous_output_calculation_state = OutputCalculationState { controller };
    let mut temporary_output_calculation_state = previous_output_calculation_state;

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
                } else if new_rc_state.mode >= 1400 && new_rc_state.mode < 1600 {
                    flight_mode = FlightMode::Acro;
                } else {
                    // manual mode
                    flight_mode = FlightMode::Manual;
                }
                // Might want to switch to manual mode if AHRS update times out (without throttle?)
            }
            ControlTaskEvent::AhrsState(new_ahrs_state) => {
                ahrs_state = new_ahrs_state;
                // On AHRS sample, propagate the output state
                previous_output_calculation_state = temporary_output_calculation_state;

                if let Some(dt) = Mono::now().checked_duration_since(rc_timestamp) {
                    if rc_state.armed && dt.to_millis() > 500 {
                        flight_mode = FlightMode::Failsafe;
                    }
                }
            }
        }

        if cx.shared.flags.reset_ahrs.load(SeqCst) {
            // Don't really like this yet
            previous_output_calculation_state.controller.reset();
        }

        let (output_command, output_state) = calculate_output(
            &ahrs_state,
            &rc_state,
            &flight_mode,
            previous_output_calculation_state,
        )
        .await;
        pwm_output_sender.try_send(output_command).ok();

        // save the state
        temporary_output_calculation_state = output_state;
    }
}

#[derive(Copy, Clone)]
struct OutputCalculationState {
    controller: Controller,
}

async fn calculate_output(
    ahrs_state: &AhrsState,
    rc_state: &RcState,
    flight_mode: &FlightMode,
    mut state: OutputCalculationState,
) -> (OutputCommand, OutputCalculationState) {
    // This is called at least every AHRS update.
    // It also re-runs when new RC state or flight mode is set, but it will then re-calculate
    // the outputs and state. This allows the super fast response to changes in flight mode or RC
    // Otherwise we'd have to wait for a new AHRS sample to actually change the output
    // To prevent infinite loops, we don't allow to change flight mode in here. That should be
    // calculated separately
    // The state will only propagate when a new AHRS sample is provided.

    // This function is not yet triggered by flight mode changes in itself, unless caused by rc_state

    let output_command = match flight_mode {
        FlightMode::Manual => OutputCommand {
            armed: rc_state.armed,
            roll: rc_state.roll,
            pitch: rc_state.pitch,
            throttle: rc_state.throttle,
            yaw: rc_state.yaw,
        },
        FlightMode::Stabilized => {
            let pitch_offset = if rc_state.pitch_offset > 1500 {
                -(rc_state.pitch_offset as i16 - 1500)
            } else {
                0
            };
            let roll_setpoint = ((rc_state.roll as i16 - 1500) as f32 / 500.0) * 45.0 * PI / 180.0;
            let pitch_setpoint =
                ((rc_state.pitch as i16 + pitch_offset - 1500) as f32 / 500.0) * -30.0 * PI / 180.0;
            let pitch_offset = 2.0 * PI / 180.0; // 2 degrees pitch up by default
            let ControllerOutput { roll, pitch } = state.controller.update(
                ahrs_state,
                roll_setpoint,
                pitch_offset + pitch_setpoint,
                rc_state.armed,
            );
            OutputCommand {
                armed: rc_state.armed,
                roll: ((roll * 500.0) as i16 + 1500) as u16,
                pitch: ((pitch * 500.0) as i16 + 1500) as u16,
                throttle: rc_state.throttle,
                yaw: rc_state.yaw,
            }
        }
        FlightMode::Acro => {
            // 180deg/s roll, 90 deg/s pitch
            let roll_rate_setpoint =
                ((rc_state.roll as i16 - 1500) as f32 / 500.0) * 180.0 * PI / 180.0;
            let pitch_rate_setpoint =
                ((rc_state.pitch as i16 - 1500) as f32 / 500.0) * -90.0 * PI / 180.0;

            let ControllerOutput { roll, pitch } = state.controller.stabilize_rates(
                ahrs_state,
                roll_rate_setpoint,
                pitch_rate_setpoint,
                rc_state.armed,
            );
            OutputCommand {
                armed: rc_state.armed,
                roll: ((roll * 500.0) as i16 + 1500) as u16,
                pitch: ((pitch * 500.0) as i16 + 1500) as u16,
                throttle: rc_state.throttle,
                yaw: rc_state.yaw,
            }
        }
        FlightMode::Failsafe => {
            let ControllerOutput { roll, pitch } =
                state.controller.update(ahrs_state, 0.0, 0.0, true);
            OutputCommand {
                armed: false,
                roll: ((roll * 500.0) as i16 + 1500) as u16,
                pitch: ((pitch * 500.0) as i16 + 1500) as u16,
                throttle: 900,
                yaw: 1500,
            }
        }
    };

    (output_command, state)
}
