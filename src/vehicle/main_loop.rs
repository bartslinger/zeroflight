use crate::app::Mono;
use crate::common::{
    ActuatorPwmCommands, ImuData, OutputCommand, RcState, TimestampedValue, Value, PI,
};
use crate::vehicle::ahrs::Ahrs;
use crate::vehicle::attitude_control::{AttitudeController, ControllerOutput};
use crate::vehicle::mixing::output_mixing;
use crate::vehicle::modes::Mode;
use crate::vehicle::radio::{radio_mapping, RcCommand, ThreePositionSwitch};
use rtic_monotonics::Monotonic;

pub struct MainState {
    armed: bool,
    ahrs: Ahrs,
    rc_command: TimestampedValue<RcCommand>,
    attitude_controller: AttitudeController,
}

impl Default for MainState {
    fn default() -> Self {
        MainState {
            armed: false,
            ahrs: Ahrs::new(),
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
/// # Design Decisions
///
/// I want the main loop to be easily understandable. The RC values are raw PWM. It is up to the
/// implementation of this function to create a more high-level abstraction.
/// The IMU data is 'unfiltered', except for filtering that happens on the IMU itself. The units are
/// in m/s^2 and rad/s.
///
/// The outputs are also in PWM. So it is up this function to do the mixing and scaling.
///
pub fn main_loop(
    state: &mut MainState,
    mode: &mut Mode,
    imu_update: &TimestampedValue<ImuData>,
    rc: &mut Value<RcState>,
) -> ActuatorPwmCommands {
    let now = Mono::now();

    // RC mapping (update only if new RC state is available)
    if let Some(rc_state) = rc.updated() {
        let rc_command = radio_mapping(rc_state.value(), &mut state.armed);
        state.rc_command.update(rc_command);
    }
    let rc_command = state.rc_command.value();

    // Activate failsafe if armed and no RC signal for 500ms
    let rc_timed_out = now
        .checked_duration_since(state.rc_command.timestamp)
        .map(|dt| dt.to_millis() > 500)
        .unwrap_or(true);
    if state.armed && rc_timed_out {
        *mode = Mode::Failsafe;
    }

    // Update AHRS based on IMU data
    if rc_command.ahrs_reset_switch {
        state.attitude_controller.reset();
        state.ahrs.reset();
    }
    let ahrs_state = state.ahrs.imu_update(imu_update.value());

    // Update (flight) mode
    *mode = match rc_command.mode_switch {
        ThreePositionSwitch::Low => Mode::Stabilized,
        ThreePositionSwitch::Middle => Mode::Acro,
        ThreePositionSwitch::High => Mode::Manual,
    };

    // Run controller
    if !state.armed {
        state.attitude_controller.reset();
    }
    let output_command = match mode {
        Mode::Manual => OutputCommand {
            armed: state.armed,
            roll: rc_command.roll,
            pitch: rc_command.pitch,
            throttle: rc_command.throttle,
            yaw: rc_command.yaw,
        },
        Mode::Stabilized => {
            let pitch_offset = if rc_command.pitch_offset > 0.0 {
                -rc_command.pitch_offset
            } else {
                0.0
            };
            let roll_setpoint = rc_command.roll * 45.0 * PI / 180.0;
            let pitch_setpoint = (rc_command.pitch + pitch_offset) * -30.0 * PI / 180.0;
            let pitch_level_setpoint = 2.0 * PI / 180.0; // 2 degrees pitch up by default
            let ControllerOutput { roll, pitch } = state.attitude_controller.update(
                ahrs_state,
                roll_setpoint,
                pitch_level_setpoint + pitch_setpoint,
                state.armed,
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
                ahrs_state,
                roll_rate_setpoint,
                pitch_rate_setpoint,
                state.armed,
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
            let ControllerOutput { roll, pitch } =
                state.attitude_controller.update(ahrs_state, 0.0, 0.0, true);
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
