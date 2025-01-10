use crate::common::{AhrsState, ImuData};
use dcmimu::DCMIMU;

pub struct Ahrs {
    state: AhrsState,
    dcmimu: DCMIMU,
}

impl Default for Ahrs {
    fn default() -> Self {
        Self::new()
    }
}

impl Ahrs {
    pub fn new() -> Self {
        Ahrs {
            state: AhrsState {
                angles: dcmimu::EulerAngles {
                    roll: 0.0,
                    pitch: 0.0,
                    yaw: 0.0,
                },
            },
            dcmimu: DCMIMU::new(),
        }
    }

    pub fn imu_update(&mut self, imu_data: &ImuData, dt: f32) -> &AhrsState {
        let (dcm, _gyro_bias) = self
            .dcmimu
            .update(imu_data.rates, imu_data.acceleration, dt); // because we only use every 10th sample

        self.state.angles = dcm;
        &self.state
    }

    pub fn reset(&mut self) {
        self.dcmimu = DCMIMU::new();
    }

    pub fn state(&self) -> &AhrsState {
        &self.state
    }
}
