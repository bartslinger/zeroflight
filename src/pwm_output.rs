pub(crate) async fn pwm_output_task(
    cx: crate::app::pwm_output_task::Context<'_>,
    mut pwm_output_receiver: rtic_sync::channel::Receiver<'static, crate::OutputCommand, 1>,
) {
    use crate::app::Mono;
    use rtic_monotonics::systick::prelude::*;
    use rtic_monotonics::Monotonic;
    defmt::info!("pwm output task started");
    loop {
        match Mono::timeout_after(100.millis(), pwm_output_receiver.recv()).await {
            Ok(v) => {
                let rc_state = v.unwrap();
                // defmt::info!(
                //     "channel info package ready {} {} {} {} {}",
                //     rc_state.armed,
                //     rc_state.roll,
                //     rc_state.pitch,
                //     rc_state.throttle,
                //     rc_state.yaw,
                // );
                if rc_state.armed {
                    let throttle = rc_state.throttle.min(2000).max(1000);
                    cx.local.s1.set_duty(throttle);
                } else {
                    cx.local.s1.set_duty(900);
                };

                // scale roll 60%
                let s3_duty = ((rc_state.roll as i16 - 1500) * 6 / 10 * -1 + 1500) as u16;
                let s4_duty = ((rc_state.roll as i16 - 1500) * 6 / 10 * -1 + 1500) as u16;

                let s5_duty = ((rc_state.pitch as i16 - 1500) * -1 + 1500) as u16;
                let s6_duty = rc_state.pitch;

                cx.local.s3.set_duty(s3_duty.min(1800).max(1200));
                cx.local.s4.set_duty(s4_duty.min(1800).max(1200));
                cx.local.s5.set_duty(s5_duty.min(2000).max(1000));
                cx.local.s6.set_duty(s6_duty.min(2000).max(1000));
            }
            Err(_) => {
                defmt::error!("pwm output timeout");
                cx.local.s1.set_duty(900);
                cx.local.s3.set_duty(1500);
                cx.local.s4.set_duty(1500);
                cx.local.s5.set_duty(1500);
                cx.local.s6.set_duty(1500);
            }
        }
    }
}
