use super::modes::Mode;
use crate::common::{AhrsState, OutputCommand, RcState, PI};
use crate::vehicle::attitude_control::{AttitudeController, ControllerOutput};

// Needs to be Copy/Clone
// Don't change the name
#[derive(Copy, Clone)]
pub struct OutputCalculationState {
    pub controller: AttitudeController,
}

pub async fn calculate_output(
    ahrs_state: &AhrsState,
    rc_state: &RcState,
    flight_mode: &Mode,
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
        Mode::Manual => OutputCommand {
            armed: rc_state.armed,
            roll: rc_state.roll,
            pitch: rc_state.pitch,
            throttle: rc_state.throttle,
            yaw: rc_state.yaw,
        },
        Mode::Stabilized => {
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
        Mode::Acro => {
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
        Mode::Failsafe => {
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
