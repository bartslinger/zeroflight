use super::attitude_control::{AttitudeController, ControllerOutput};
use super::mixing::output_mixing;
use super::modes::Mode;
use super::radio::{radio_mapping, RcCommand, ThreePositionSwitch};
use crate::app::Mono;
use crate::common::{
    ActuatorPwmCommands, AhrsState, ImuData, MaybeUpdated, OutputCommand, RcState, TimestampedValue, PI,
};
use rtic_monotonics::Monotonic;

pub struct MainState {
    armed: bool,
    mode: Mode,
    rc_command: TimestampedValue<RcCommand>,
    attitude_controller: AttitudeController,
}

impl Default for MainState {
    fn default() -> Self {
        MainState {
            armed: false,
            mode: Mode::Stabilized,
            rc_command: TimestampedValue::new(RcCommand::default()),
            attitude_controller: AttitudeController::new(),
        }
    }
}

/// Main loop for the vehicle
///
/// This function runs every time a new IMU update is available. This is typically published
/// at 1000Hz but could be different based on the configuration.
///
/// Because the AHRS is slow (needs more than 1ms to calculate), the attitude is calculated at a
/// lower frequency (250Hz) in a separate task and provided as input to this function.
///
/// # Design Decisions
///
/// I want the main loop to be easily understandable. The RC values are raw PWM. It is up to the
/// implementation of this function to create a more high-level abstraction (RcCommand).
/// The IMU data is 'unfiltered', except for filtering that happens on the IMU itself. The units are
/// in m/s^2 and rad/s.
///
/// The outputs are also in PWM. So it is up this function to do the mixing and scaling.
///
/// This function is also responsible for safety such as arming and failsafe behavior for
/// RC timeouts.
///
pub fn main_loop(
    state: &mut MainState,
    imu_data: &ImuData,
    ahrs_state: &mut MaybeUpdated<AhrsState>,
    rc_state: &mut MaybeUpdated<RcState>,
    dt: f32,
) -> ActuatorPwmCommands {
    let now = Mono::now();

    // Use AHRS state, just get the value. This is updated at 250Hz so data might be a bit 'old'.
    // Angles don't change that fast so it's fine. We still use 1000Hz for rates from IMU.
    let ahrs_state = ahrs_state.read().value();

    // RC mapping (update only if new RC state is available)
    if let Some(new_rc_state) = rc_state.updated() {
        let rc_command = radio_mapping(new_rc_state.value(), &mut state.armed);
        state.rc_command.set(rc_command, new_rc_state.timestamp);
    }
    let rc_command = state.rc_command.value();

    let rc_timed_out = now
        .checked_duration_since(state.rc_command.timestamp)
        .map(|dt| dt.to_millis() > 500)
        .unwrap_or(true);

    // Update (flight) mode
    if state.armed && rc_timed_out {
        // Activate failsafe if armed and no RC signal for 500ms
        state.mode = Mode::Failsafe;
        // I chose not to disarm now, so it will respond immediately when RC becomes available again
    } else {
        state.mode = match rc_command.mode_switch {
            ThreePositionSwitch::Low => Mode::Stabilized,
            ThreePositionSwitch::Middle => Mode::Acro,
            ThreePositionSwitch::High => Mode::Manual,
        };
    }

    if rc_command.ahrs_reset_switch {
        state.attitude_controller.reset();
    }

    // Run controller
    if !state.armed {
        state.attitude_controller.reset();
    }
    let output_command = match state.mode {
        Mode::Manual => OutputCommand {
            armed: state.armed,
            roll: rc_command.roll,
            pitch: rc_command.pitch,
            throttle: rc_command.throttle,
            yaw: rc_command.yaw,
        },
        Mode::Stabilized => {
            let rc_pitch_offset = if rc_command.pitch_offset > 0.0 {
                // This is used to add pitch up command for a very simple sort of launch mode
                -rc_command.pitch_offset
            } else {
                0.0
            };
            let roll_setpoint = rc_command.roll * 45.0 * PI / 180.0;
            let pitch_setpoint = (rc_command.pitch + rc_pitch_offset) * -30.0 * PI / 180.0;
            let pitch_level_setpoint = 2.0 * PI / 180.0; // 2 degrees pitch up by default
            let ControllerOutput { roll, pitch } = state.attitude_controller.update(
                ahrs_state,
                imu_data,
                roll_setpoint,
                pitch_level_setpoint + pitch_setpoint,
                state.armed,
                dt,
            );
            OutputCommand {
                armed: state.armed,
                roll,
                pitch,
                throttle: rc_command.throttle,
                yaw: rc_command.yaw,
            }
        }
        Mode::Acro => {
            // 180deg/s roll, 90 deg/s pitch
            let roll_rate_setpoint = rc_command.roll * 180.0 * PI / 180.0;
            let pitch_rate_setpoint = rc_command.pitch * -90.0 * PI / 180.0;

            let ControllerOutput { roll, pitch } = state.attitude_controller.stabilize_rates(
                imu_data,
                roll_rate_setpoint,
                pitch_rate_setpoint,
                state.armed,
                dt,
            );
            OutputCommand {
                armed: state.armed,
                roll,
                pitch,
                throttle: rc_command.throttle,
                yaw: rc_command.yaw,
            }
        }
        Mode::Failsafe => {
            let ControllerOutput { roll, pitch } = state
                .attitude_controller
                .update(ahrs_state, imu_data, 0.0, 0.0, true, dt);
            OutputCommand {
                armed: false,
                roll,
                pitch,
                throttle: 0.0,
                yaw: 0.0,
            }
        }
    };

    // Mix and scale outputs
    let actuator_commands = output_mixing(&output_command);

    // Publish commands
    actuator_commands
}
