use crate::common::{ImuData, PI};
use crate::IMU_DATA_CHANNEL;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::Spi;
use embassy_time::{Duration, Instant, Timer};
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
    let sender = IMU_DATA_CHANNEL.sender();
    let mut hit: u32 = 0;
    let mut miss: u32 = 0;

    loop {
        // get fifo count
        let start = Instant::now();
        let fifo_count = crate::icm42688p::get_fifo_count(&mut spi_dev).await;

        if fifo_count > 0 {
            hit += 1;
            if fifo_count > 1 {
                defmt::info!("    [high] fifo count: {}", fifo_count);
            }
            // if hit % 100 == 0 {
            //     defmt::info!(
            //         "    [high] fifo count: {}, {}, {}, {}",
            //         fifo_count,
            //         hit,
            //         miss,
            //         hit as f32 / (hit + miss) as f32
            //     );
            // }
            // get imu data
            let imu_data = crate::icm42688p::get_fifo_sample(&mut spi_dev).await;
            let imu_data = parse_imu_data(&imu_data[1..17]);
            sender.send(imu_data);

            if fifo_count == 1 {
                // might just as well sleep now
                // Ideally you want to be just slightly ahead of the fifo, so next fifo count is
                // ideally still 0
                // Then when you query the fifo count again, ideally it is 1
                // Ideal hit/miss ratio is therefore 0.5
                let next = start + Duration::from_micros(900);
                Timer::at(next).await;
            }
            // defmt::info!("    [high] imu data: {:?}", imu_data);
        } else {
            miss += 1;
        }
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
        timestamp_unscaled: raw_timestamp,
        acceleration: (acc_x, acc_y, acc_z),
        rates: (gyro_x, gyro_y, gyro_z),
    }
}
