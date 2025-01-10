// use crate::common::{AhrsState, PI};
// use crate::IMUDATAPOOL;
// use heapless::pool::boxed::Box;
//
// pub(crate) async fn ahrs_task(
//     cx: crate::app::ahrs_task::Context<'_>,
//     mut imu_data_receiver: rtic_sync::channel::Receiver<'static, Box<IMUDATAPOOL>, 1>,
//     mut ahrs_state_sender: rtic_sync::channel::Sender<'static, AhrsState, 1>,
// ) {
//     defmt::info!("imu handler spawned");
//     let mut ahrs = dcmimu::DCMIMU::new();
//     let mut i = 0;
//     let mut prev_timestamp = 0;
//     while let Ok(buf) = imu_data_receiver.recv().await {
//         i += 1;
//         if i % 10 != 0 {
//             // only do 100Hz, this imu estimator is slow
//             continue;
//         }
//         let raw_acc_x = i16::from_be_bytes([buf[1], buf[2]]);
//         let raw_acc_y = i16::from_be_bytes([buf[3], buf[4]]);
//         let raw_acc_z = i16::from_be_bytes([buf[5], buf[6]]);
//         let raw_gyro_x = i16::from_be_bytes([buf[7], buf[8]]);
//         let raw_gyro_y = i16::from_be_bytes([buf[9], buf[10]]);
//         let raw_gyro_z = i16::from_be_bytes([buf[11], buf[12]]);
//         let raw_temperature = buf[13];
//         let raw_timestamp = u16::from_be_bytes([buf[14], buf[15]]);
//         let acc_x = raw_acc_x as f32 * 9.80665 / 2048.0;
//         let acc_y = -raw_acc_y as f32 * 9.80665 / 2048.0;
//         let acc_z = raw_acc_z as f32 * 9.80665 / 2048.0;
//         let gyro_x = -raw_gyro_x as f32 * PI / 180.0 / 16.4;
//         let gyro_y = raw_gyro_y as f32 * PI / 180.0 / 16.4;
//         let gyro_z = -raw_gyro_z as f32 * PI / 180.0 / 16.4;
//         let _temperature_celsius = (raw_temperature as f32 / 2.07) + 25.0;
//         let timestamp: u16 = (raw_timestamp as u32 * 32 / 30) as u16;
//         let _diff = timestamp.wrapping_sub(prev_timestamp);
//         prev_timestamp = timestamp;
//
//         let reset_ahrs = &cx.shared.flags.reset_ahrs;
//         if reset_ahrs.load(core::sync::atomic::Ordering::SeqCst) {
//             ahrs = dcmimu::DCMIMU::new();
//         }
//
//         let (dcm, _gyro_bias) = ahrs.update((gyro_x, gyro_y, gyro_z), (acc_x, acc_y, acc_z), 0.01);
//
//         if let Err(_) = ahrs_state_sender.try_send(AhrsState {
//             angles: dcm,
//             rates: (gyro_x, gyro_y, gyro_z),
//             _acceleration: (acc_x, acc_y, acc_z),
//         }) {
//             // defmt::error!("error publishing ahrs state");
//         }
//         if i % 1000 == 0 {
//             // defmt::info!("gyro x: {}\ty: {}\tz: {}", gyro_x, gyro_y, gyro_z);
//             // defmt::info!("accel x: {}\ty: {}\tz: {}", acc_x, acc_y, acc_z);
//             // defmt::info!(
//             //     "T: {}\troll: {}\tpitch: {}\tyaw: {}",
//             //     diff,
//             //     dcm.roll * 180.0 / PI,
//             //     dcm.pitch * 180.0 / PI,
//             //     dcm.yaw * 180.0 / PI
//             // );
//             i = 0;
//         }
//     }
// }
