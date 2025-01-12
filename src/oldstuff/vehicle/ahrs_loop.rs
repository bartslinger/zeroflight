use crate::common::{AhrsState, ImuData};
use crate::vehicle::ahrs::Ahrs;

#[derive(Default)]
pub struct AhrsLoopState {
    ahrs: Ahrs,
}

/// AHRS loop
///
/// This function is called only every 4th IMU update. With an IMU update rate of 1kHz, this
/// means that this function is called at 250Hz.
///
pub fn ahrs_loop(state: &mut AhrsLoopState, imu_update: &ImuData, dt: f32) -> AhrsState {
    state.ahrs.imu_update(imu_update, dt);
    state.ahrs.state().clone()
}
