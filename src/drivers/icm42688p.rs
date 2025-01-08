use crate::boards::board::{
    ImuCsPin, ImuDmaRxStream, ImuDmaRxTransfer, ImuDmaTxStream, ImuDmaTxTransfer, ImuSpiDev,
};

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

pub(crate) struct Icm42688p<SPIDEV, CS> {
    spi: SPIDEV,
    cs: CS,
}

impl<SPIDEV: embedded_hal::spi::SpiBus, CS: embedded_hal::digital::OutputPin>
    Icm42688p<SPIDEV, CS>
{
    pub(crate) fn new(spi: SPIDEV, mut cs: CS) -> Self {
        cs.set_high().ok();
        Self { spi, cs }
    }

    pub(crate) fn init(&mut self, delay: &mut stm32f4xx_hal::timer::SysDelay) {
        use embedded_hal::delay::DelayNs;
        self.cs.set_low().ok();
        delay.delay_us(10);

        let whoami = self.read_register(REG_WHO_AM_I);
        if whoami != 0x47 {
            defmt::error!("IMU not found");
            panic!("IMU not found");
        }

        // disable power on accel and gyro for configuration (see datasheet 12.9)
        self.write_register(REG_PWR_MGMT0, 0x00);
        delay.delay_us(10);
        self.write_register(REG_FIFO_CONFIG, 0x80); // FIFO_CONFIG STOP-on-full
        delay.delay_us(10);
        self.write_register(REG_FIFO_CONFIG1, 0x07); // FIFO_CONFIG1 enable temp, accel, gyro
        delay.delay_us(10);
        self.write_register(REG_INTF_CONFIG0, 0xF0); // big Endian, count records, hold last sample
        delay.delay_us(10);
        self.write_register(REG_SIGNAL_PATH_RESET, 0x02); // flush the FIFO
        delay.delay_us(10);
        self.write_register(REG_GYRO_CONFIG0, 0x06); // gyro 1kHz ODR
        delay.delay_us(10);
        self.write_register(REG_ACCEL_CONFIG0, 0x06); // accel 1kHz ODR
        delay.delay_us(10);

        self.write_register(REG_REG_BANK_SEL, 1);
        delay.delay_us(10);
        let aaf_enable = self.read_register(REG_GYRO_CONFIG_STATIC2);
        delay.delay_us(10);
        self.write_register(REG_GYRO_CONFIG_STATIC2, aaf_enable & !0x03); // enable not and AAF
        delay.delay_us(10);
        self.write_register(REG_GYRO_CONFIG_STATIC3, 6); // 258Hz gyro bandwith
        delay.delay_us(10);
        self.write_register(REG_GYRO_CONFIG_STATIC4, 36); // 258Hz gyro bandwith
        delay.delay_us(10);
        self.write_register(
            REG_GYRO_CONFIG_STATIC5,
            (10 << 4) & 0xF0, /* | ((36 >> 8) & 0x0F) */
        ); // 258Hz gyro bandwith
        delay.delay_us(10);

        self.write_register(REG_REG_BANK_SEL, 2);
        delay.delay_us(10);
        self.write_register(REG_ACCEL_CONFIG_STATIC2, 5 << 1); // 213Hz accel bandwith
        delay.delay_us(10);
        self.write_register(REG_ACCEL_CONFIG_STATIC3, 25); // 213Hz accel bandwith
        delay.delay_us(10);
        self.write_register(
            REG_ACCEL_CONFIG_STATIC4,
            (10 << 4) & 0xF0, /* | ((25 >> 8) & 0x0F) */
        ); // 213Hz accel bandwith

        self.write_register(REG_REG_BANK_SEL, 0);
        delay.delay_us(10);

        /*
          From Ardupilot:
          fix for the "stuck gyro" issue, which affects all IxM42xxx
          sensors. This disables the AFSR feature which changes the
          noise sensitivity with angular rate. When the switch happens
          (at around 100 deg/sec) the gyro gets stuck for around 2ms,
          producing constant output which causes a DC gyro bias
        */
        let v = self.read_register(REG_INTF_CONFIG1);
        delay.delay_us(10);
        self.write_register(REG_INTF_CONFIG1, (v & 0x3F) | 0x40);
        delay.delay_us(10);

        self.write_register(REG_PWR_MGMT0, 0x0F); // enable power on accel and gyro
                                                  // min 200us sleep recommended
        delay.delay_us(300);
    }

    fn read_register(&mut self, reg: u8) -> u8 {
        self.cs.set_low().ok();
        let mut buf = [reg | 0x80, 0x00];
        self.spi
            .transfer_in_place(&mut buf)
            .expect("IMU write failed");
        self.cs.set_high().ok();
        buf[1]
    }

    fn write_register(&mut self, reg: u8, value: u8) {
        self.cs.set_low().ok();
        let mut buf = [reg, value];
        self.spi
            .transfer_in_place(&mut buf)
            .expect("IMU write failed");
        self.cs.set_high().ok();
    }

    pub(crate) fn release(self) -> (SPIDEV, CS) {
        (self.spi, self.cs)
    }
}

pub(crate) struct Icm42688pDmaContext<TX, RX, CS> {
    pub(crate) tx_transfer: TX,
    pub(crate) rx_transfer: RX,
    pub(crate) tx_buffer: Option<&'static mut [u8; 129]>,
    pub(crate) rx_buffer: Option<&'static mut [u8; 129]>,
    pub(crate) cs: CS,
}

pub(crate) fn start_dma(
    tx_buffer_1: &'static mut [u8; 129],
    rx_buffer_1: &'static mut [u8; 129],
    tx_buffer_2: &'static mut [u8; 129],
    rx_buffer_2: &'static mut [u8; 129],
    spi: ImuSpiDev,
    tx_stream: ImuDmaTxStream,
    rx_stream: ImuDmaRxStream,
    mut cs: ImuCsPin,
) -> Icm42688pDmaContext<ImuDmaTxTransfer, ImuDmaRxTransfer, ImuCsPin> {
    tx_buffer_1[0] = 0x2E | 0x80;
    tx_buffer_2[0] = 0x2E | 0x80;

    let (tx, rx) = spi.use_dma().txrx();
    let mut rx_transfer = stm32f4xx_hal::dma::Transfer::init_peripheral_to_memory(
        rx_stream,
        rx,
        rx_buffer_1,
        Some(3),
        None,
        stm32f4xx_hal::dma::config::DmaConfig::default()
            .memory_increment(true)
            .transfer_complete_interrupt(true),
    );
    let mut tx_transfer = stm32f4xx_hal::dma::Transfer::init_memory_to_peripheral(
        tx_stream,
        tx,
        tx_buffer_1,
        Some(3),
        None,
        stm32f4xx_hal::dma::config::DmaConfig::default().memory_increment(true),
    );

    // start
    cs.set_low();
    // starting rx_transfer before tx_transfer seems more robust
    // (works with even large delay between the two calls)
    rx_transfer.start(|_| {});
    tx_transfer.start(|_| {});

    let icm42688p_dma_context = Icm42688pDmaContext {
        tx_transfer,
        rx_transfer,
        rx_buffer: Some(rx_buffer_2),
        tx_buffer: Some(tx_buffer_2),
        cs,
    };
    icm42688p_dma_context
}
