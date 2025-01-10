use crate::common::{AhrsState, ImuData, TimestampedValue};
use dcmimu::DCMIMU;

pub struct Ahrs {
    imu_update_counter: u8,
    state: AhrsState,
    dcmimu: DCMIMU,
}

impl Ahrs {
    pub fn new() -> Self {
        Ahrs {
            imu_update_counter: 0,
            state: AhrsState {
                angles: dcmimu::EulerAngles {
                    roll: 0.0,
                    pitch: 0.0,
                    yaw: 0.0,
                },
                rates: (0.0, 0.0, 0.0),
                _acceleration: (0.0, 0.0, 0.0),
            },
            dcmimu: DCMIMU::new(),
        }
    }

    pub fn imu_update(&mut self, imu_update: &TimestampedValue<ImuData>, dt: f32) -> &AhrsState {
        if self.imu_update_counter % 10 == 0 {
            self.imu_update_counter = 0;
            let imu_value = imu_update.value();
            let (dcm, _gyro_bias) =
                self.dcmimu
                    .update(imu_value.rates, imu_value.acceleration, 0.1 * dt); // because we only use every 10th sample

            self.state.angles = dcm;
            self.state.rates = imu_value.rates;
        }
        // TODO: Could propagate the gyro's on the angles between the dcmimu updates
        self.imu_update_counter += 1;
        &self.state
    }

    pub fn reset(&mut self) {
        self.dcmimu = DCMIMU::new();
    }

    // pub fn state(&self) -> &AhrsState {
    //     &self.state
    // }
}
