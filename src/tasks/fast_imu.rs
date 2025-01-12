use crate::WHO_AM_I_RESPONSE_CHANNEL;
use defmt::unwrap;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::Spi;
use embassy_time::Timer;
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_bus::spi::NoDelay;

#[embassy_executor::task]
pub async fn run_fast_imu_task(
    mut spi_dev: embedded_hal_bus::spi::ExclusiveDevice<
        Spi<'static, Async>,
        Output<'static>,
        NoDelay,
    >,
) {
    let sender = WHO_AM_I_RESPONSE_CHANNEL.sender();
    let mut count: u8 = 0;

    loop {
        // Do a SPI whoami
        let reg = 0x75;
        let mut buf: [u8; 2] = [reg | 0x80, 0x00];
        defmt::info!("    [high] transfer start");
        // cs.set_low();
        if let Err(e) = spi_dev.transfer_in_place(&mut buf).await {
            defmt::error!("    [high] error: {}", e);
            continue;
        }
        // cs.set_high();
        defmt::info!("    [high] response: {:02x}", buf[1]);
        sender.send(count);
        defmt::info!("    [high] byte was sent");
        count += 1;

        Timer::after_millis(1000).await;
    }
}
