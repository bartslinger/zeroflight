#![no_main]
#![no_std]
//#![deny(warnings)]
#![deny(unsafe_code)]
//#![deny(missing_docs)]

use panic_probe as _;

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {
    use stm32f4xx_hal::gpio::{self, Output, PushPull};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: gpio::PA13<Output<PushPull>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        use stm32f4xx_hal::prelude::*;

        // Setup clocks
        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();

        let gpioa = cx.device.GPIOA.split();
        let led = gpioa.pa13.into_push_pull_output();

        (Shared {}, Local { led })
    }
}
