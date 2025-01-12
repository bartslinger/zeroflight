use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::Spi;
use embassy_time::Timer;
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_bus::spi::NoDelay;

const REG_WHO_AM_I: u8 = 0x75;
const REG_PWR_MGMT0: u8 = 0x4E;
const REG_FIFO_CONFIG: u8 = 0x16;
const REG_FIFO_CONFIG1: u8 = 0x5F;
const REG_INTF_CONFIG0: u8 = 0x4C;
const REG_INTF_CONFIG1: u8 = 0x4D;
const REG_SIGNAL_PATH_RESET: u8 = 0x4B;
const REG_GYRO_CONFIG0: u8 = 0x4F;
const REG_ACCEL_CONFIG0: u8 = 0x50;
const REG_REG_BANK_SEL: u8 = 0x76;
const REG_GYRO_CONFIG_STATIC2: u8 = 0x0B;
const REG_GYRO_CONFIG_STATIC3: u8 = 0x0C;
const REG_GYRO_CONFIG_STATIC4: u8 = 0x0D;
const REG_GYRO_CONFIG_STATIC5: u8 = 0x0E;
const REG_ACCEL_CONFIG_STATIC2: u8 = 0x03;
const REG_ACCEL_CONFIG_STATIC3: u8 = 0x04;
const REG_ACCEL_CONFIG_STATIC4: u8 = 0x05;

pub async fn read_register(
    spi_dev: &mut embedded_hal_bus::spi::ExclusiveDevice<
        Spi<'static, Async>,
        Output<'static>,
        NoDelay,
    >,
    reg: u8,
) -> u8 {
    let mut buf = [reg | 0x80, 0x00];
    spi_dev.transfer_in_place(&mut buf).await.ok();
    buf[1]
}

pub async fn write_register(
    spi_dev: &mut embedded_hal_bus::spi::ExclusiveDevice<
        Spi<'static, Async>,
        Output<'static>,
        NoDelay,
    >,
    reg: u8,
    val: u8,
) {
    let mut buf = [reg, val];
    spi_dev.transfer_in_place(&mut buf).await.ok();
}

pub async fn init(
    spi_dev: &mut embedded_hal_bus::spi::ExclusiveDevice<
        Spi<'static, Async>,
        Output<'static>,
        NoDelay,
    >,
) {
    let whoami = read_register(spi_dev, REG_WHO_AM_I).await;
    if whoami != 0x47 {
        defmt::error!("IMU not found");
        panic!("IMU not found");
    }

    // disable power on accel and gyro for configuration (see datasheet 12.9)
    write_register(spi_dev, REG_PWR_MGMT0, 0x00).await;
    Timer::after_micros(10).await;
    write_register(spi_dev, REG_FIFO_CONFIG, 0x80).await; // FIFO_CONFIG STOP-on-full
    Timer::after_micros(10).await;
    write_register(spi_dev, REG_FIFO_CONFIG1, 0x07).await; // FIFO_CONFIG1 enable temp, accel, gyro
    Timer::after_micros(10).await;
    write_register(spi_dev, REG_INTF_CONFIG0, 0xF0).await; // big Endian, count records, hold last sample
    Timer::after_micros(10).await;
    write_register(spi_dev, REG_SIGNAL_PATH_RESET, 0x02).await; // flush the FIFO
    Timer::after_micros(10).await;
    write_register(spi_dev, REG_GYRO_CONFIG0, 0x06).await; // gyro 1kHz ODR
    Timer::after_micros(10).await;
    write_register(spi_dev, REG_ACCEL_CONFIG0, 0x06).await; // accel 1kHz ODR
    Timer::after_micros(10).await;

    write_register(spi_dev, REG_REG_BANK_SEL, 1).await;
    Timer::after_micros(10).await;
    let aaf_enable = read_register(spi_dev, REG_GYRO_CONFIG_STATIC2).await;
    Timer::after_micros(10).await;
    write_register(spi_dev, REG_GYRO_CONFIG_STATIC2, aaf_enable & !0x03).await; // enable not and AAF
    Timer::after_micros(10).await;
    write_register(spi_dev, REG_GYRO_CONFIG_STATIC3, 6).await; // 258Hz gyro bandwith
    Timer::after_micros(10).await;
    write_register(spi_dev, REG_GYRO_CONFIG_STATIC4, 36).await; // 258Hz gyro bandwith
    Timer::after_micros(10).await;
    write_register(
        spi_dev,
        REG_GYRO_CONFIG_STATIC5,
        (10 << 4) & 0xF0, /* | ((36 >> 8) & 0x0F) */
    )
    .await; // 258Hz gyro bandwith
    Timer::after_micros(10).await;

    write_register(spi_dev, REG_REG_BANK_SEL, 2).await;
    Timer::after_micros(10).await;
    write_register(spi_dev, REG_ACCEL_CONFIG_STATIC2, 5 << 1).await; // 213Hz accel bandwith
    Timer::after_micros(10).await;
    write_register(spi_dev, REG_ACCEL_CONFIG_STATIC3, 25).await; // 213Hz accel bandwith
    Timer::after_micros(10).await;
    write_register(
        spi_dev,
        REG_ACCEL_CONFIG_STATIC4,
        (10 << 4) & 0xF0, /* | ((25 >> 8) & 0x0F) */
    )
    .await; // 213Hz accel bandwith

    write_register(spi_dev, REG_REG_BANK_SEL, 0).await;
    Timer::after_micros(10).await;

    /*
      From Ardupilot:
      fix for the "stuck gyro" issue, which affects all IxM42xxx
      sensors. This disables the AFSR feature which changes the
      noise sensitivity with angular rate. When the switch happens
      (at around 100 deg/sec) the gyro gets stuck for around 2ms,
      producing constant output which causes a DC gyro bias
    */
    let v = read_register(spi_dev, REG_INTF_CONFIG1).await;
    Timer::after_micros(10).await;
    write_register(spi_dev, REG_INTF_CONFIG1, (v & 0x3F) | 0x40).await;
    Timer::after_micros(10).await;

    write_register(spi_dev, REG_PWR_MGMT0, 0x0F).await; // enable power on accel and gyro
                                                        // min 200us sleep recommended
    Timer::after_micros(300).await;
}
