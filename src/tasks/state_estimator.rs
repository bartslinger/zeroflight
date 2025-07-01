use crate::common::{AhrsState, PI};
use crate::IMU_DATA_CHANNEL;

#[embassy_executor::task]
pub async fn run_ahrs_task() {
    let receiver = IMU_DATA_CHANNEL.receiver();
    // let mut previous_timestamp_unscaled: u16 = 0;

    let mut ahrs = AhrsState::default();
    let mut dcmimu = dcmimu::DCMIMU::new();

    let mut counter: u32 = 0;

    loop {
        counter += 1;
        // It comes in at 1000 Hz, but we process at 250 Hz
        // So we throw away 3 out of 4 samples
        // Could improve the performance by processing all samples and 'filter' to 250 Hz
        let _ = receiver.receive().await; // throw away
        let _ = receiver.receive().await; // throw away
        let _ = receiver.receive().await; // throw away
        let imu_data = receiver.receive().await;

        // let diff = imu_data
        //     .timestamp_unscaled
        //     .wrapping_sub(previous_timestamp_unscaled);
        // let diff_micros = diff as u32 * 32 / 30;
        // if imu_data.timestamp_unscaled < previous_timestamp_unscaled {
        //     defmt::info!(
        //         "[low] Received IMU: {}\t->\t{}\t..\t{}",
        //         previous_timestamp_unscaled,
        //         imu_data.timestamp_unscaled,
        //         diff_micros,
        //     );
        // }
        // previous_timestamp_unscaled = imu_data.timestamp_unscaled;

        // This is a long blocking call, takes somewhere in between 1.5 and 2 ms
        let (angles, _gyro_bias) = dcmimu.update(imu_data.rates, imu_data.acceleration, 0.004);
        ahrs.angles = angles;
        if counter % 250 == 0 {
            defmt::info!(
                "    [low] roll: {}, pitch: {}",
                ahrs.angles.roll * 180.0 / PI,
                ahrs.angles.pitch * 180.0 / PI
            );
        }
    }
}
