use crate::common::{ActuatorCommands, AhrsState, ImuData, RcState, Update};
use crate::vehicle::ahrs::Ahrs;
use crate::vehicle::radio::radio_mapping;

pub struct MainLoopState {
    ahrs: Ahrs,
}

impl Default for MainLoopState {
    fn default() -> Self {
        MainLoopState { ahrs: Ahrs::new() }
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
    state: &mut MainLoopState,
    imu_update: &ImuData,
    rc: &Update<RcState>,
) -> ActuatorCommands {
    // Update AHRS based on IMU data
    let ahrs_state = state.ahrs.imu_update(imu_update);

    // Parse RC input
    let radio = radio_mapping(rc.value());

    // Update (flight) mode

    // Run attitude controller

    // Publish commands
    todo!()
}
