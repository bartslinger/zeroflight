use embedded_hal_nb::nb;

pub(crate) fn crsf_interrupt_handler<SERIAL>(
    serial: &mut SERIAL,
    crsf_data_sender: &mut rtic_sync::channel::Sender<'static, u8, 64>,
) where
    SERIAL: embedded_hal_nb::serial::Read,
{
    use embedded_hal_nb::serial::Error;
    let byte = match serial.read() {
        Ok(v) => v,
        Err(nb::Error::Other(e)) => {
            defmt::info!("Error in receiving crsf byte: {}", e.kind() as u32);
            return;
        }
        Err(nb::Error::WouldBlock) => {
            defmt::info!("Error in receiving crsf byte: would block");
            return;
        }
    };
    if let Err(_e) = crsf_data_sender.try_send(byte) {
        defmt::error!("send error from crsf interrupt");
    };
}
