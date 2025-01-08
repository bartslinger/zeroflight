use crate::boards::board::CrsfSerial;
use stm32f4xx_hal::nb;

pub(crate) fn crsf_interrupt_handler(
    serial: &mut CrsfSerial,
    crsf_data_sender: &mut rtic_sync::channel::Sender<'static, u8, 64>,
) {
    use stm32f4xx_hal::prelude::*;
    if serial.is_rx_not_empty() {
        let byte = match serial.read() {
            Ok(v) => v,
            Err(nb::Error::Other(e)) => {
                defmt::info!("Error in receiving crsf byte: {}", e as u32);
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
}
