use crate::boards::board;
use crate::common::{ImuData, PI};
use crate::drivers::icm42688p::Icm42688pDmaContext;
use crate::IMUDATAPOOL;
use heapless::pool::boxed::Box;

pub(crate) fn icm42688_interrupt_handler(
    dma_context: &mut Icm42688pDmaContext<
        board::ImuDmaTxTransfer,
        board::ImuDmaRxTransfer,
        board::ImuCsPin,
    >,
    prev_fifo_count: &mut u16,
    imu_data_sender: &mut rtic_sync::channel::Sender<'static, Box<IMUDATAPOOL>, 1>,
) {
    use stm32f4xx_hal::dma::traits::StreamISR;
    let Icm42688pDmaContext {
        tx_transfer,
        rx_transfer,
        tx_buffer,
        rx_buffer,
        cs,
    } = dma_context;

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
        let parsed_imu_data = parse_imu_data(&filled_rx_buffer[1..17]);
        if let Ok(output) = IMUDATAPOOL.alloc(parsed_imu_data) {
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
    *prev_fifo_count = fifo_count;
    if let Some(imu_data) = imu_data.take() {
        if let Err(_e) = imu_data_sender.try_send(imu_data) {
            // defmt::info!("IMU data sender failed");
        };
    }
}

fn parse_imu_data(buf: &[u8]) -> ImuData {
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
    let _timestamp: u16 = (raw_timestamp as u32 * 32 / 30) as u16;

    ImuData {
        acceleration: (acc_x, acc_y, acc_z),
        rates: (gyro_x, gyro_y, gyro_z),
    }
}
