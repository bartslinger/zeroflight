pub(crate) fn imu_rx_handler(cx: crate::app::imu_rx_handler::Context<'_>) {
    use stm32f4xx_hal::dma::traits::StreamISR;
    let cs = &mut cx.local.icm42688p_dma_context.cs;
    let rx_transfer = &mut cx.local.icm42688p_dma_context.rx_transfer;
    let tx_transfer = &mut cx.local.icm42688p_dma_context.tx_transfer;
    let tx_buffer = &mut cx.local.icm42688p_dma_context.tx_buffer;
    let rx_buffer = &mut cx.local.icm42688p_dma_context.rx_buffer;

    cs.set_high(); // stop the transaction

    rx_transfer.clear_transfer_complete();
    let ready_rx_buffer = rx_buffer.take().unwrap();
    let ready_tx_buffer = tx_buffer.take().unwrap();
    // cortex_m::asm::delay(10_000);
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
        let mut output = [0u8; 16];
        output.copy_from_slice(&filled_rx_buffer[1..17]);
        imu_data = Some(output);
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
        cx.local.imu_data_sender.try_send(imu_data).ok();
    }
}
