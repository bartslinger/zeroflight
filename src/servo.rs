pub struct Servo<PWM: embedded_hal::pwm::SetDutyCycle> {
    pwm_channel: PWM,
}

impl<PWM: embedded_hal::pwm::SetDutyCycle> Servo<PWM> {
    pub fn new(pwm_channel: PWM) -> Self {
        Self { pwm_channel }
    }

    pub fn set_position(&mut self, position: u16) {
        self.pwm_channel.set_duty_cycle(position).ok();
    }
}
