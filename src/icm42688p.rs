use stm32f4xx_hal::gpio::{Output, PushPull};

pub(crate) trait CsPin {
    fn set_high(&mut self);
    fn set_low(&mut self);
}

impl<const P: char, const N: u8> CsPin for stm32f4xx_hal::gpio::Pin<P, N, Output<PushPull>> {
    fn set_high(&mut self) {
        self.set_high();
    }

    fn set_low(&mut self) {
        self.set_low();
    }
}

pub(crate) struct Icm42688p<CS: CsPin> {
    spi: stm32f4xx_hal::spi::Spi<stm32f4xx_hal::pac::SPI1, false, u8>,
    cs: CS,
}

impl<CS: CsPin> Icm42688p<CS> {
    pub(crate) fn new(spi: stm32f4xx_hal::spi::Spi<stm32f4xx_hal::pac::SPI1>, mut cs: CS) -> Self {
        cs.set_high();
        Self { spi, cs }
    }

    pub(crate) fn init(&mut self, delay: &mut stm32f4xx_hal::timer::SysDelay) {
        use embedded_hal::delay::DelayNs;
        self.cs.set_low();
        delay.delay_us(10);

        // disable power on accel and gyro for configuration (see datasheet 12.9)
        self.cs.set_low();
        let mut buf = [0x4E, 0x00];
        self.spi
            .transfer_in_place(&mut buf)
            .expect("IMU write failed");
        self.cs.set_high();
        // min 200us sleep recommended
        delay.delay_us(300);

        // configure the FIFO
        self.cs.set_low();
        let mut buf = [0x16, 0x80]; // FIFO_CONFIG STOP-on-full
        self.spi
            .transfer_in_place(&mut buf)
            .expect("IMU write failed");
        self.cs.set_high();
        delay.delay_us(300);

        self.cs.set_low();
        let mut buf = [0x5F, 0x07]; // FIFO_CONFIG1 enable temp, accel, gyro
        self.spi
            .transfer_in_place(&mut buf)
            .expect("IMU write failed");
        self.cs.set_high();
        delay.delay_us(300);

        self.cs.set_low();
        let mut buf = [0x4C, 0xF0]; // big Endian, count records, hold last sample
        self.spi
            .transfer_in_place(&mut buf)
            .expect("IMU write failed");
        self.cs.set_high();
        delay.delay_us(300);

        self.cs.set_low();
        let mut buf = [0x4B, 0x02]; // SIGNAL_PATH_RESET flush the FIFO
        self.spi
            .transfer_in_place(&mut buf)
            .expect("IMU write failed");
        self.cs.set_high();
        delay.delay_us(300);

        // // read power bits
        // self.cs.set_low();
        // let mut buf = [0x4E | 0x80, 0x00];
        // self.spi.transfer_in_place(&mut buf).expect("IMU write failed");
        // self.cs.set_high();
        // defmt::info!("Power bits: {:02x}", buf[1]);
        // delay.delay_ms(15);

        // set accel range
        self.cs.set_low();
        let mut buf = [0x50, (0x00 << 5) | (0x06 & 0x0F)];
        self.spi
            .transfer_in_place(&mut buf)
            .expect("IMU write failed");
        self.cs.set_high();
        delay.delay_ms(15);

        // enable power on accel and gyro
        self.cs.set_low();
        let mut buf = [0x4E, 0x0F];
        self.spi
            .transfer_in_place(&mut buf)
            .expect("IMU write failed");
        self.cs.set_high();
        // min 200us sleep recommended
        delay.delay_us(300);

        // try a two-step who-am-i
        self.cs.set_low();
        let mut buf = [0x75 | 0x80];
        self.spi
            .transfer_in_place(&mut buf)
            .expect("IMU write failed");
        defmt::info!("was 0x75: {:02x}", buf[0]);
        let mut buf = [0x00];
        self.spi
            .transfer_in_place(&mut buf)
            .expect("IMU write failed");
        defmt::info!("who am i: {:02x}", buf[0]);
        self.cs.set_high();
    }

    pub(crate) fn start_dma(
        mut self,
        tx_stream: stm32f4xx_hal::dma::StreamX<stm32f4xx_hal::pac::DMA2, 3>,
        rx_stream: stm32f4xx_hal::dma::StreamX<stm32f4xx_hal::pac::DMA2, 0>,
        tx_buffer_1: &'static mut [u8; 129],
        rx_buffer_1: &'static mut [u8; 129],
        tx_buffer_2: &'static mut [u8; 129],
        rx_buffer_2: &'static mut [u8; 129],
    ) -> Icm42688pDmaContext<CS> {
        tx_buffer_1[0] = 0x2E | 0x80;
        tx_buffer_2[0] = 0x2E | 0x80;

        let (tx, rx) = self.spi.use_dma().txrx();
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
        self.cs.set_low();
        // starting rx_transfer before tx_transfer seems more robust
        // (works with even large delay between the two calls)
        rx_transfer.start(|_| {});
        tx_transfer.start(|_| {});

        Icm42688pDmaContext {
            tx_transfer,
            rx_transfer,
            rx_buffer: Some(rx_buffer_2),
            tx_buffer: Some(tx_buffer_2),
            cs: self.cs,
        }
    }
}

type TxTransfer = stm32f4xx_hal::dma::Transfer<
    stm32f4xx_hal::dma::Stream3<stm32f4xx_hal::pac::DMA2>,
    3,
    stm32f4xx_hal::spi::Tx<stm32f4xx_hal::pac::SPI1>,
    stm32f4xx_hal::dma::MemoryToPeripheral,
    &'static mut [u8; 129],
>;

type RxTransfer = stm32f4xx_hal::dma::Transfer<
    stm32f4xx_hal::dma::Stream0<stm32f4xx_hal::pac::DMA2>,
    3,
    stm32f4xx_hal::spi::Rx<stm32f4xx_hal::pac::SPI1>,
    stm32f4xx_hal::dma::PeripheralToMemory,
    &'static mut [u8; 129],
>;
pub(crate) struct Icm42688pDmaContext<CS: CsPin> {
    pub(crate) tx_transfer: TxTransfer,
    pub(crate) rx_transfer: RxTransfer,
    pub(crate) tx_buffer: Option<&'static mut [u8; 129]>,
    pub(crate) rx_buffer: Option<&'static mut [u8; 129]>,
    pub(crate) cs: CS,
}
