use crate::app::Mono;
use rtic_monotonics::systick::prelude::*;

pub(crate) async fn blink_task(_cx: crate::app::blink_task::Context<'_>) {
    defmt::info!("blink task");
    let mut instant = Mono::now();
    loop {
        instant += 1000.millis();
        Mono::delay_until(instant).await;
        // cx.local.led.set_high();
        // let now = Mono::now();
        // defmt::info!("LED off at {:?}", now.ticks());

        instant += 1000.millis();
        Mono::delay_until(instant).await;
        // cx.local.led.set_low();
        // let now = Mono::now();
        // defmt::info!("LED on  at {:?}", now.ticks());
    }
}
