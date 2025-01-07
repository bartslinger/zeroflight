use stm32f4xx_hal::gpio;
use stm32f4xx_hal::pac::{
    ADC1, ADC2, ADC3, ADC_COMMON, CAN1, CAN2, CRC, CRYP, DAC, DBGMCU, DCMI, DMA1, DMA2, EXTI,
    FLASH, FPU, FPU_CPACR, FSMC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI, GPIOJ, GPIOK, HASH,
    I2C1, I2C2, I2C3, I2S2EXT, I2S3EXT, IWDG, LTDC, NVIC_STIR, OTG_FS_DEVICE, OTG_FS_GLOBAL,
    OTG_FS_HOST, OTG_FS_PWRCLK, OTG_HS_DEVICE, OTG_HS_GLOBAL, OTG_HS_HOST, OTG_HS_PWRCLK, PWR, RNG,
    RTC, SAI1, SCB_ACTRL, SDIO, SPI1, SPI2, SPI3, SPI4, SPI5, SPI6, STK, SYSCFG, TIM1, TIM10,
    TIM11, TIM12, TIM13, TIM14, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM8, TIM9, UART4, UART5,
    USART1, USART2, USART3, USART6, WWDG,
};
use stm32f4xx_hal::timer::PwmChannel;

//     DEF_TIM(TIM4,   CH2, PB7,  TIM_USE_OUTPUT_AUTO,   1, 0), // S1 D(1,3,2)
//     DEF_TIM(TIM4,   CH1, PB6,  TIM_USE_OUTPUT_AUTO,   1, 0), // S2 D(1,0,2)
//
//     DEF_TIM(TIM3,   CH3, PB0,  TIM_USE_OUTPUT_AUTO,   1, 0), // S3 D(1,7,5)
//     DEF_TIM(TIM3,   CH4, PB1,  TIM_USE_OUTPUT_AUTO,   1, 0), // S4 D(1,2,5)
//     DEF_TIM(TIM8,   CH3, PC8,  TIM_USE_OUTPUT_AUTO,   1, 0), // S5 D(2,4,7)
//     DEF_TIM(TIM8,   CH4, PC9,  TIM_USE_OUTPUT_AUTO,   1, 0), // S6 D(2,7,7)
pub type S1 = PwmChannel<TIM4, 1>;
pub type S2 = PwmChannel<TIM4, 0>;
pub type S3 = PwmChannel<TIM3, 2>;
pub type S4 = PwmChannel<TIM3, 3>;
pub type S5 = PwmChannel<TIM8, 2>;
pub type S6 = PwmChannel<TIM8, 3>;

pub struct PwmOutputs {
    pub s1: S1,
    pub s2: S2,
    pub s3: S3,
    pub s4: S4,
    pub s5: S5,
    pub s6: S6,
}

pub struct I2cPins<SCL, SDA> {
    pub scl: SCL,
    pub sda: SDA,
}

pub struct UsartPins<TX, RX> {
    pub tx: TX,
    pub rx: RX,
}

pub struct SpiPins<SCK, MISO, MOSI, CS> {
    pub sck: SCK,
    pub miso: MISO,
    pub mosi: MOSI,
    pub cs: CS,
}

pub struct UsbPins<DP, DM> {
    pub dp: DP,
    pub dm: DM,
}

pub type CrsfSerial = stm32f4xx_hal::serial::Serial<stm32f4xx_hal::pac::USART1, u8>;

// For RTIC
#[doc(hidden)]
pub use stm32f4xx_hal::pac::interrupt;
#[doc(hidden)]
pub use stm32f4xx_hal::pac::Peripherals;
#[doc(hidden)]
pub use stm32f4xx_hal::pac::NVIC_PRIO_BITS;
use stm32f4xx_hal::rcc::Clocks;

#[allow(non_snake_case)]
#[allow(unused)]
pub struct Board {
    pub clocks: Clocks,
    pub pwm_outputs: PwmOutputs,
    pub i2c1: I2cPins<gpio::PB8, gpio::PB9>,
    pub usart1: UsartPins<gpio::PA9, gpio::PA10>,
    pub spi1: SpiPins<gpio::PA5, gpio::PA6, gpio::PA7, gpio::PA4>,
    pub usb_pins: UsbPins<gpio::PA12, gpio::PA11>,
    ///RNG
    pub RNG: RNG,
    ///DCMI
    pub DCMI: DCMI,
    ///FSMC
    pub FSMC: FSMC,
    ///DBGMCU
    pub DBGMCU: DBGMCU,
    ///DMA2
    pub DMA2: DMA2,
    ///DMA1
    pub DMA1: DMA1,
    ///RCC
    // pub RCC: RCC,
    ///GPIOI
    pub GPIOI: GPIOI,
    ///GPIOH
    pub GPIOH: GPIOH,
    ///GPIOG
    pub GPIOG: GPIOG,
    ///GPIOF
    pub GPIOF: GPIOF,
    ///GPIOE
    pub GPIOE: GPIOE,
    ///GPIOD
    pub GPIOD: GPIOD,
    ///GPIOC
    // pub GPIOC: GPIOC,
    ///GPIOJ
    pub GPIOJ: GPIOJ,
    ///GPIOK
    pub GPIOK: GPIOK,
    ///GPIOB
    // pub GPIOB: GPIOB,
    ///GPIOA
    // pub GPIOA: GPIOA,
    ///SYSCFG
    pub SYSCFG: SYSCFG,
    ///SPI1
    pub SPI1: SPI1,
    ///SPI2
    pub SPI2: SPI2,
    ///SPI3
    pub SPI3: SPI3,
    ///I2S2ext
    pub I2S2EXT: I2S2EXT,
    ///I2S3ext
    pub I2S3EXT: I2S3EXT,
    ///SPI4
    pub SPI4: SPI4,
    ///SPI5
    pub SPI5: SPI5,
    ///SPI6
    pub SPI6: SPI6,
    ///SDIO
    pub SDIO: SDIO,
    ///ADC1
    pub ADC1: ADC1,
    ///ADC2
    pub ADC2: ADC2,
    ///ADC3
    pub ADC3: ADC3,
    ///USART1
    pub USART1: USART1,
    ///USART6
    pub USART6: USART6,
    ///USART2
    pub USART2: USART2,
    ///USART3
    pub USART3: USART3,
    ///DAC
    pub DAC: DAC,
    ///PWR
    pub PWR: PWR,
    ///I2C1
    pub I2C1: I2C1,
    ///I2C3
    pub I2C3: I2C3,
    ///I2C2
    pub I2C2: I2C2,
    ///IWDG
    pub IWDG: IWDG,
    ///WWDG
    pub WWDG: WWDG,
    ///RTC
    pub RTC: RTC,
    ///UART4
    pub UART4: UART4,
    ///UART5
    pub UART5: UART5,
    ///ADC_Common
    pub ADC_COMMON: ADC_COMMON,
    ///TIM1
    pub TIM1: TIM1,
    ///TIM8
    // pub TIM8: TIM8,
    ///TIM2
    pub TIM2: TIM2,
    ///TIM3
    // pub TIM3: TIM3,
    ///TIM4
    // pub TIM4: TIM4,
    ///TIM5
    pub TIM5: TIM5,
    ///TIM9
    pub TIM9: TIM9,
    ///TIM12
    pub TIM12: TIM12,
    ///TIM10
    pub TIM10: TIM10,
    ///TIM13
    pub TIM13: TIM13,
    ///TIM14
    pub TIM14: TIM14,
    ///TIM11
    pub TIM11: TIM11,
    ///TIM6
    pub TIM6: TIM6,
    ///TIM7
    pub TIM7: TIM7,
    ///CRC
    pub CRC: CRC,
    ///OTG_FS_GLOBAL
    pub OTG_FS_GLOBAL: OTG_FS_GLOBAL,
    ///OTG_FS_HOST
    pub OTG_FS_HOST: OTG_FS_HOST,
    ///OTG_FS_DEVICE
    pub OTG_FS_DEVICE: OTG_FS_DEVICE,
    ///OTG_FS_PWRCLK
    pub OTG_FS_PWRCLK: OTG_FS_PWRCLK,
    ///CAN1
    pub CAN1: CAN1,
    ///CAN2
    pub CAN2: CAN2,
    ///FLASH
    pub FLASH: FLASH,
    ///EXTI
    pub EXTI: EXTI,
    ///OTG_HS_GLOBAL
    pub OTG_HS_GLOBAL: OTG_HS_GLOBAL,
    ///OTG_HS_HOST
    pub OTG_HS_HOST: OTG_HS_HOST,
    ///OTG_HS_DEVICE
    pub OTG_HS_DEVICE: OTG_HS_DEVICE,
    ///OTG_HS_PWRCLK
    pub OTG_HS_PWRCLK: OTG_HS_PWRCLK,
    ///SAI1
    pub SAI1: SAI1,
    ///LTDC
    pub LTDC: LTDC,
    ///HASH
    pub HASH: HASH,
    ///CRYP
    pub CRYP: CRYP,
    ///FPU
    pub FPU: FPU,
    ///STK
    pub STK: STK,
    ///NVIC_STIR
    pub NVIC_STIR: NVIC_STIR,
    ///FPU_CPACR
    pub FPU_CPACR: FPU_CPACR,
    ///SCB_ACTRL
    pub SCB_ACTRL: SCB_ACTRL,
}

impl Board {}

impl From<Peripherals> for Board {
    fn from(p: Peripherals) -> Self {
        use stm32f4xx_hal::prelude::*;
        let rcc = p.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(168.MHz()).require_pll48clk().freeze();
        defmt::info!("SYSCLK: {}", clocks.sysclk().raw());
        defmt::info!("HCLK: {}", clocks.hclk().raw());
        defmt::info!("PCLK1: {}", clocks.pclk1().raw());
        defmt::info!("PCLK2: {}", clocks.pclk2().raw());
        assert!(clocks.is_pll48clk_valid());

        let gpioa = p.GPIOA.split();
        let gpiob = p.GPIOB.split();
        let gpioc = p.GPIOC.split();

        let (_, (_, _, tim3_ch3, tim3_ch4)) = p.TIM3.pwm_us(20_000.micros(), &clocks);
        let (_, (tim4_ch1, tim4_ch2, _, _)) = p.TIM4.pwm_us(20_000.micros(), &clocks);

        let (_, (_, _, tim8_ch3, tim8_ch4)) = p.TIM8.pwm_us(20_000.micros(), &clocks);

        let mut s1 = tim4_ch2.with(gpiob.pb7);
        let mut s2 = tim4_ch1.with(gpiob.pb6);
        let mut s3 = tim3_ch3.with(gpiob.pb0);
        let mut s4 = tim3_ch4.with(gpiob.pb1);
        let mut s5 = tim8_ch3.with(gpioc.pc8);
        let mut s6 = tim8_ch4.with(gpioc.pc9);

        s1.set_duty(900);
        s1.enable();

        s2.set_duty(900);
        s2.enable();

        s3.set_duty(1500);
        s3.enable();

        s4.set_duty(1500);
        s4.enable();

        s5.set_duty(1500);
        s5.enable();

        s6.set_duty(1500);
        s6.enable();

        let i2c1 = I2cPins {
            scl: gpiob.pb8,
            sda: gpiob.pb9,
        };

        let pwm_outputs = PwmOutputs {
            s1,
            s2,
            s3,
            s4,
            s5,
            s6,
        };

        let usart1 = UsartPins {
            tx: gpioa.pa9,
            rx: gpioa.pa10,
        };
        // let usart1 = board.USART1;

        let spi1 = SpiPins {
            sck: gpioa.pa5,
            miso: gpioa.pa6,
            mosi: gpioa.pa7,
            cs: gpioa.pa4,
        };

        let usb_pins = UsbPins {
            dp: gpioa.pa12,
            dm: gpioa.pa11,
        };

        Self {
            clocks,
            pwm_outputs,
            i2c1,
            usart1,
            spi1,
            usb_pins,
            RNG: p.RNG,
            DCMI: p.DCMI,
            FSMC: p.FSMC,
            DBGMCU: p.DBGMCU,
            DMA2: p.DMA2,
            DMA1: p.DMA1,
            // RCC: p.RCC,
            GPIOI: p.GPIOI,
            GPIOH: p.GPIOH,
            GPIOG: p.GPIOG,
            GPIOF: p.GPIOF,
            GPIOE: p.GPIOE,
            GPIOD: p.GPIOD,
            // GPIOC: p.GPIOC,
            GPIOJ: p.GPIOJ,
            GPIOK: p.GPIOK,
            // GPIOB: p.GPIOB,
            // GPIOA: p.GPIOA,
            SYSCFG: p.SYSCFG,
            SPI1: p.SPI1,
            SPI2: p.SPI2,
            SPI3: p.SPI3,
            I2S2EXT: p.I2S2EXT,
            I2S3EXT: p.I2S3EXT,
            SPI4: p.SPI4,
            SPI5: p.SPI5,
            SPI6: p.SPI6,
            SDIO: p.SDIO,
            ADC1: p.ADC1,
            ADC2: p.ADC2,
            ADC3: p.ADC3,
            USART1: p.USART1,
            USART6: p.USART6,
            USART2: p.USART2,
            USART3: p.USART3,
            DAC: p.DAC,
            PWR: p.PWR,
            I2C1: p.I2C1,
            I2C3: p.I2C3,
            I2C2: p.I2C2,
            IWDG: p.IWDG,
            WWDG: p.WWDG,
            RTC: p.RTC,
            UART4: p.UART4,
            UART5: p.UART5,
            ADC_COMMON: p.ADC_COMMON,
            TIM1: p.TIM1,
            // TIM8: p.TIM8,
            TIM2: p.TIM2,
            // TIM3: p.TIM3,
            // TIM4: p.TIM4,
            TIM5: p.TIM5,
            TIM9: p.TIM9,
            TIM12: p.TIM12,
            TIM10: p.TIM10,
            TIM13: p.TIM13,
            TIM14: p.TIM14,
            TIM11: p.TIM11,
            TIM6: p.TIM6,
            TIM7: p.TIM7,
            CRC: p.CRC,
            OTG_FS_GLOBAL: p.OTG_FS_GLOBAL,
            OTG_FS_HOST: p.OTG_FS_HOST,
            OTG_FS_DEVICE: p.OTG_FS_DEVICE,
            OTG_FS_PWRCLK: p.OTG_FS_PWRCLK,
            CAN1: p.CAN1,
            CAN2: p.CAN2,
            FLASH: p.FLASH,
            EXTI: p.EXTI,
            OTG_HS_GLOBAL: p.OTG_HS_GLOBAL,
            OTG_HS_HOST: p.OTG_HS_HOST,
            OTG_HS_DEVICE: p.OTG_HS_DEVICE,
            OTG_HS_PWRCLK: p.OTG_HS_PWRCLK,
            SAI1: p.SAI1,
            LTDC: p.LTDC,
            HASH: p.HASH,
            CRYP: p.CRYP,
            FPU: p.FPU,
            STK: p.STK,
            NVIC_STIR: p.NVIC_STIR,
            FPU_CPACR: p.FPU_CPACR,
            SCB_ACTRL: p.SCB_ACTRL,
        }
    }
}
