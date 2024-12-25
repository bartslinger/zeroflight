#![no_main]
#![no_std]
//#![deny(warnings)]
#![deny(unsafe_code)]
//#![deny(missing_docs)]

use defmt_rtt as _;
use panic_halt as _;

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [USART1])]
mod app {
    use stm32f1xx_hal::gpio::{self, Output, PushPull};
    // use cortex_m_semihosting::{debug, hprintln};
    use rtic_monotonics::systick::prelude::*;

    systick_monotonic!(Mono, 100);

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: gpio::PC13<Output<PushPull>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        use stm32f1xx_hal::prelude::*;

        Mono::start(cx.core.SYST, 72_000_000);
        defmt::info!("clock setup");
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();
        let _clocks = rcc
            .cfgr
            .use_hse(8.MHz()) // Use an external 8 MHz crystal oscillator
            .sysclk(72.MHz()) // Set system clock to 72 MHz
            .pclk1(36.MHz()) // Set APB1 clock (max 36 MHz)
            .pclk2(72.MHz()) // Set APB2 clock (max 72 MHz)
            .freeze(&mut flash.acr);

        let mut gpioc = cx.device.GPIOC.split();
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_low();

        defmt::info!("init done");

        (Shared {}, Local { led })
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        blink::spawn().ok();

        loop {
            // defmt::info!("loop");
            cortex_m::asm::nop();
        }
    }

    #[task(priority = 1, local = [led])]
    async fn blink(cx: blink::Context) {
        defmt::info!("blink task");
        loop {
            Mono::delay(1000.millis()).await;
            cx.local.led.set_high();
            defmt::info!("LED off");
            Mono::delay(1000.millis()).await;
            cx.local.led.set_low();
            defmt::info!("LED on");
        }
    }
}
