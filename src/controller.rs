// pub struct ControllerInput {
//     pitch_angle_rad: f32,
//     roll_angle_rad: f32,
// }

pub struct ControllerOutput {
    pub pitch: f32,
    pub roll: f32,
}

pub struct Controller {}

impl Controller {
    pub fn new() -> Self {
        Controller {}
    }

    pub fn update(
        &self,
        measurement: dcmimu::EulerAngles,
        roll_setpoint_rad: f32,
        pitch_setpoint_rad: f32,
    ) -> ControllerOutput {
        let pitch_error = pitch_setpoint_rad - measurement.pitch;
        let roll_error = roll_setpoint_rad - measurement.roll;

        let pitch_output = pitch_error * -2.0;
        let roll_output = roll_error * 2.0;

        ControllerOutput {
            pitch: pitch_output,
            roll: roll_output,
        }
    }
}
