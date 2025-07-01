use crate::common::ActuatorPwmCommands;

pub(crate) async fn pwm_output_task(
    cx: crate::app::pwm_output_task::Context<'_>,
    mut pwm_output_receiver: rtic_sync::channel::Receiver<'static, ActuatorPwmCommands, 1>,
) {
    use crate::app::Mono;
    use rtic_monotonics::systick::prelude::*;
    use rtic_monotonics::Monotonic;
    defmt::info!("pwm output task started");
    loop {
        match Mono::timeout_after(100.millis(), pwm_output_receiver.recv()).await {
            Ok(Ok(command)) => {
                cx.local
                    .pwm_outputs
                    .s1
                    .set_duty(command.s1.min(2000).max(900));
                cx.local
                    .pwm_outputs
                    .s2
                    .set_duty(command.s2.min(2000).max(900));
                cx.local
                    .pwm_outputs
                    .s3
                    .set_duty(command.s3.min(2000).max(1000));
                cx.local
                    .pwm_outputs
                    .s4
                    .set_duty(command.s4.min(2000).max(1000));
                cx.local
                    .pwm_outputs
                    .s5
                    .set_duty(command.s5.min(2000).max(1000));
                cx.local
                    .pwm_outputs
                    .s6
                    .set_duty(command.s6.min(2000).max(1000));
            }
            Ok(Err(_)) | Err(_) => {
                defmt::error!("pwm output timeout");
                cx.local.pwm_outputs.s1.set_duty(900);
                cx.local.pwm_outputs.s2.set_duty(900);
                cx.local.pwm_outputs.s3.set_duty(1500);
                cx.local.pwm_outputs.s4.set_duty(1500);
                cx.local.pwm_outputs.s5.set_duty(1500);
                cx.local.pwm_outputs.s6.set_duty(1500);
            }
        }
    }
}
