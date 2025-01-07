use crate::servo::Servo;
use stm32f4xx_hal::pac::{
    ADC1, ADC2, ADC3, ADC_COMMON, CAN1, CAN2, CRC, CRYP, DAC, DBGMCU, DCMI, DMA1, DMA2, EXTI,
    FLASH, FPU, FPU_CPACR, FSMC, GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI,
    GPIOJ, GPIOK, HASH, I2C1, I2C2, I2C3, I2S2EXT, I2S3EXT, IWDG, LTDC, NVIC_STIR, OTG_FS_DEVICE,
    OTG_FS_GLOBAL, OTG_FS_HOST, OTG_FS_PWRCLK, OTG_HS_DEVICE, OTG_HS_GLOBAL, OTG_HS_HOST,
    OTG_HS_PWRCLK, PWR, RCC, RNG, RTC, SAI1, SCB_ACTRL, SDIO, SPI1, SPI2, SPI3, SPI4, SPI5, SPI6,
    STK, SYSCFG, TIM1, TIM10, TIM11, TIM12, TIM13, TIM14, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM8,
    TIM9, UART4, UART5, USART1, USART2, USART3, USART6, WWDG,
};
use stm32f4xx_hal::timer::PwmChannel;

pub type PwmOutputs = Servo<PwmChannel<TIM4, 0>>;

pub use stm32f4xx_hal::pac::NVIC_PRIO_BITS;
pub use stm32f4xx_hal::pac::{interrupt, Peripherals};

#[allow(non_snake_case)]
pub struct Board {
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
    pub RCC: RCC,
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
    pub GPIOC: GPIOC,
    ///GPIOJ
    pub GPIOJ: GPIOJ,
    ///GPIOK
    pub GPIOK: GPIOK,
    ///GPIOB
    pub GPIOB: GPIOB,
    ///GPIOA
    pub GPIOA: GPIOA,
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
    pub TIM8: TIM8,
    ///TIM2
    pub TIM2: TIM2,
    ///TIM3
    pub TIM3: TIM3,
    ///TIM4
    pub TIM4: TIM4,
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

impl Board {
    // #[inline]
    // pub fn take() -> Option<Self> {
    //     let peripherals = stm32f4xx_hal::pac::Peripherals::take()?;
    //     Some(Self::new(peripherals))
    // }

    pub fn new(p: stm32f4xx_hal::pac::Peripherals) -> Self {
        Self {
            RNG: p.RNG,
            DCMI: p.DCMI,
            FSMC: p.FSMC,
            DBGMCU: p.DBGMCU,
            DMA2: p.DMA2,
            DMA1: p.DMA1,
            RCC: p.RCC,
            GPIOI: p.GPIOI,
            GPIOH: p.GPIOH,
            GPIOG: p.GPIOG,
            GPIOF: p.GPIOF,
            GPIOE: p.GPIOE,
            GPIOD: p.GPIOD,
            GPIOC: p.GPIOC,
            GPIOJ: p.GPIOJ,
            GPIOK: p.GPIOK,
            GPIOB: p.GPIOB,
            GPIOA: p.GPIOA,
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
            TIM8: p.TIM8,
            TIM2: p.TIM2,
            TIM3: p.TIM3,
            TIM4: p.TIM4,
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
