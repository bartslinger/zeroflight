use crate::common::PI;
use crate::IMUDATAPOOL;
use heapless::pool::boxed::Box;

#[derive(Copy, Clone)]
pub(crate) struct AhrsState {
    pub(crate) angles: dcmimu::EulerAngles,
    pub(crate) rates: (f32, f32, f32),
    pub(crate) _acceleration: (f32, f32, f32),
}

pub(crate) async fn imu_handler(
    cx: crate::app::imu_handler::Context<'_>,
    mut imu_data_receiver: rtic_sync::channel::Receiver<'static, Box<IMUDATAPOOL>, 1>,
    mut ahrs_state_sender: rtic_sync::channel::Sender<'static, AhrsState, 1>,
) {
    defmt::info!("imu handler spawned");
    let mut ahrs = dcmimu::DCMIMU::new();
    let mut i = 0;
    let mut prev_timestamp = 0;
    while let Ok(buf) = imu_data_receiver.recv().await {
        i += 1;
        if i % 10 != 0 {
            // only do 100Hz, this imu estimator is slow
            continue;
        }
        let raw_acc_x = i16::from_be_bytes([buf[1], buf[2]]);
        let raw_acc_y = i16::from_be_bytes([buf[3], buf[4]]);
        let raw_acc_z = i16::from_be_bytes([buf[5], buf[6]]);
        let raw_gyro_x = i16::from_be_bytes([buf[7], buf[8]]);
        let raw_gyro_y = i16::from_be_bytes([buf[9], buf[10]]);
        let raw_gyro_z = i16::from_be_bytes([buf[11], buf[12]]);
        let raw_temperature = buf[13];
        let raw_timestamp = u16::from_be_bytes([buf[14], buf[15]]);
        let acc_x = raw_acc_x as f32 * 9.80665 / 2048.0;
        let acc_y = -raw_acc_y as f32 * 9.80665 / 2048.0;
        let acc_z = raw_acc_z as f32 * 9.80665 / 2048.0;
        let gyro_x = -raw_gyro_x as f32 * PI / 180.0 / 16.4;
        let gyro_y = raw_gyro_y as f32 * PI / 180.0 / 16.4;
        let gyro_z = -raw_gyro_z as f32 * PI / 180.0 / 16.4;
        let _temperature_celsius = (raw_temperature as f32 / 2.07) + 25.0;
        let timestamp: u16 = (raw_timestamp as u32 * 32 / 30) as u16;
        let _diff = timestamp.wrapping_sub(prev_timestamp);
        prev_timestamp = timestamp;

        let reset_ahrs = &cx.shared.flags.reset_ahrs;
        if reset_ahrs.load(core::sync::atomic::Ordering::SeqCst) {
            ahrs = dcmimu::DCMIMU::new();
        }

        let (dcm, _gyro_bias) = ahrs.update((gyro_x, gyro_y, gyro_z), (acc_x, acc_y, acc_z), 0.01);

        if let Err(_) = ahrs_state_sender.try_send(AhrsState {
            angles: dcm,
            rates: (gyro_x, gyro_y, gyro_z),
            _acceleration: (acc_x, acc_y, acc_z),
        }) {
            // defmt::error!("error publishing ahrs state");
        }
        if i % 1000 == 0 {
            // defmt::info!("gyro x: {}\ty: {}\tz: {}", gyro_x, gyro_y, gyro_z);
            // defmt::info!("accel x: {}\ty: {}\tz: {}", acc_x, acc_y, acc_z);
            // defmt::info!(
            //     "T: {}\troll: {}\tpitch: {}\tyaw: {}",
            //     diff,
            //     dcm.roll * 180.0 / PI,
            //     dcm.pitch * 180.0 / PI,
            //     dcm.yaw * 180.0 / PI
            // );
            i = 0;
        }
    }
}
pub(crate) fn imu_rx_irq(cx: crate::app::imu_rx_irq::Context<'_>) {
    use stm32f4xx_hal::dma::traits::StreamISR;
    let cs = &mut cx.local.icm42688p_dma_context.cs;
    let rx_transfer = &mut cx.local.icm42688p_dma_context.rx_transfer;
    let tx_transfer = &mut cx.local.icm42688p_dma_context.tx_transfer;
    let tx_buffer = &mut cx.local.icm42688p_dma_context.tx_buffer;
    let rx_buffer = &mut cx.local.icm42688p_dma_context.rx_buffer;

    cs.set_high(); // stop the transaction

    rx_transfer.clear_transfer_complete();
    let ready_rx_buffer = rx_buffer.take().unwrap(); // we put it back later
    let ready_tx_buffer = tx_buffer.take().unwrap(); // we put it back later

    cs.set_low();

    rx_transfer.pause(|_| {});
    tx_transfer.pause(|_| {});

    let (filled_tx_buffer, _) = tx_transfer.swap(ready_tx_buffer);
    let (filled_rx_buffer, _) = rx_transfer.swap(ready_rx_buffer);

    let mut imu_data = None;
    let fifo_count = if filled_tx_buffer[0] == 0x2E | 0x80 {
        // Expecting a FIFO count
        let fifo_count = u16::from_be_bytes([filled_rx_buffer[1], filled_rx_buffer[2]]);
        // defmt::info!("fifo count: {}", fifo_count);
        fifo_count
    } else if filled_tx_buffer[0] == 0x30 | 0x80 {
        // Got IMU data
        if let Ok(mut output) = IMUDATAPOOL.alloc([0u8; 16]) {
            output.copy_from_slice(&filled_rx_buffer[1..17]);
            imu_data = Some(output);
        }
        // After that, request fifo count again
        filled_tx_buffer[0] = 0x2E | 0x80;
        0
    } else {
        // Request
        defmt::info!("invalid value in tx buffer {:02x}", filled_tx_buffer[0]);
        0
    };
    let len = if fifo_count > 0 {
        // Request IMU data
        filled_tx_buffer[0] = 0x30 | 0x80;
        17
    } else {
        filled_tx_buffer[0] = 0x2E | 0x80;
        3
    };

    let (prev_rx_buffer, _) = rx_transfer
        .next_transfer(filled_rx_buffer, Some(len))
        .unwrap();
    let (prev_tx_buffer, _) = tx_transfer
        .next_transfer(filled_tx_buffer, Some(len))
        .unwrap();
    *rx_buffer = Some(prev_rx_buffer);
    *tx_buffer = Some(prev_tx_buffer);
    *cx.local.prev_fifo_count = fifo_count;
    if let Some(imu_data) = imu_data.take() {
        if let Err(_e) = cx.local.imu_data_sender.try_send(imu_data) {
            // defmt::info!("IMU data sender failed");
        };
    }
}
