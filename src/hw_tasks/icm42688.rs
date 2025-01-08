use crate::boards::board;
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
    *prev_fifo_count = fifo_count;
    if let Some(imu_data) = imu_data.take() {
        if let Err(_e) = imu_data_sender.try_send(imu_data) {
            // defmt::info!("IMU data sender failed");
        };
    }
}
