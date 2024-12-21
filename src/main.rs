#![no_main]
#![no_std]
//#![deny(warnings)]
#![deny(unsafe_code)]
//#![deny(missing_docs)]

use panic_semihosting as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART1])]
mod app {
    use stm32f4xx_hal::gpio::{self, Output, PushPull};
    use cortex_m_semihosting::{debug, hprintln};
    use defmt;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: gpio::PA13<Output<PushPull>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        hprintln!("init");
        defmt::info!("Test");
        use stm32f4xx_hal::prelude::*;

        // Setup clocks
        hprintln!("clock setup");
        let rcc = cx.device.RCC.constrain();

        // let clocks = rcc.cfgr
        //     .use_hse(8.MHz()) // Use the external 8 MHz HSE
        //     .sysclk(168.MHz()) // Set system clock to 168 MHz
        //     .hclk(168.MHz())   // Set HCLK to 168 MHz
        //     .pclk1(42.MHz())   // Set APB1 clock to 42 MHz
        //     .pclk2(84.MHz())   // Set APB2 clock to 84 MHz
        //     .freeze();         // Apply and freeze the configuration
        // let clocks = rcc.cfgr.use_hsi().freeze();

        let gpioa = cx.device.GPIOA.split();
        let led = gpioa.pa13.into_push_pull_output();

        hprintln!("init done");

        (Shared {}, Local {led})
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        hprintln!("idle");
        once_off_task::spawn().ok();

        loop {
            hprintln!("loop");
            cortex_m::asm::nop();
            debug::exit(debug::EXIT_SUCCESS); // Exit QEMU simulator
        }
    }

    #[task(priority = 1)]
    async fn once_off_task(cx: once_off_task::Context) {
        hprintln!("once_off_task");
    }
}
