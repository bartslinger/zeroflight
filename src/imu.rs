pub(crate) async fn imu_handler(
    _cx: crate::app::imu_handler::Context<'_>,
    mut imu_data_receiver: rtic_sync::channel::Receiver<'static, [u8; 16], 5>,
) {
    defmt::info!("imu handler spawned");
    while let Ok(imu_data) = imu_data_receiver.recv().await {
        defmt::info!("imu data: {:?}", imu_data);
    }
}
