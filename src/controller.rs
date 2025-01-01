// pub struct ControllerInput {
//     pitch_angle_rad: f32,
//     roll_angle_rad: f32,
// }

use crate::imu::AhrsState;
const PI: f32 = 3.14159265358979323846264338327950288_f32;

pub struct ControllerOutput {
    pub roll: f32,
    pub pitch: f32,
}

pub struct Controller {
    roll_integral: f32,
    pitch_integral: f32,
}

impl Controller {
    pub fn new() -> Self {
        Controller {
            roll_integral: 0.0,
            pitch_integral: 0.0,
        }
    }

    pub fn update(
        &mut self,
        measurement: AhrsState,
        roll_setpoint_rad: f32,
        pitch_setpoint_rad: f32,
    ) -> ControllerOutput {
        let roll_error = roll_setpoint_rad - measurement.angles.roll;
        let pitch_error = pitch_setpoint_rad - measurement.angles.pitch;

        // INAV: fw_p_level = 20
        // with their weird scaling factor, that is 3.05 deg/s per degree

        // With a simple P-controller, calculate rate errors
        let roll_rate_setpoint = roll_error * 3.05;
        let pitch_rate_setpoint = pitch_error * 3.05;

        let roll_rate_error = roll_rate_setpoint - measurement.rates.0;
        let pitch_rate_error = pitch_rate_setpoint - measurement.rates.1;

        // INAV GAINS
        // set fw_p_pitch = 15
        // set fw_i_pitch = 5
        // set fw_d_pitch = 5
        // set fw_ff_pitch = 107
        // set fw_p_roll = 15
        // set fw_i_roll = 3
        // set fw_d_roll = 7
        // set fw_ff_roll = 60
        // transformed by the multipliers
        // fw_p_pitch = 15 / 31.0 = 0.48387096774
        // fw_i_pitch = 5 / 4.0 = 1.25
        // fw_d_pitch = 5 / 1905.0 = 0.00262237762238
        // fw_ff_pitch = 107 / 31.0 = 3.45161290323
        // fw_p_roll = 15 / 31.0 = 0.48387096774
        // fw_i_roll = 3 / 4.0 = 0.75
        // fw_d_roll = 7 / 1905.0 = 0.00367521367521
        // fw_ff_roll = 60 / 31.0 = 1.93548387097

        let fw_p_pitch = 0.48387096774 * 180.0 / PI;
        let fw_i_pitch = 1.25 * 180.0 / PI;
        // let fw_d_pitch = 0.00262237762238;
        let fw_ff_pitch = 3.45161290323 * 180.0 / PI;
        let fw_p_roll = 0.48387096774 * 180.0 / PI;
        let fw_i_roll = 0.75 * 180.0 / PI;
        // let fw_d_roll = 0.00367521367521;
        let fw_ff_roll = 1.93548387097 * 180.0 / PI;

        self.roll_integral = (self.roll_integral + roll_rate_error * fw_i_roll * 0.01)
            .min(200.0)
            .max(-200.0);
        self.pitch_integral = (self.pitch_integral + pitch_rate_error * fw_i_pitch * 0.01)
            .min(200.0)
            .max(-200.0);

        // Apply only P and FF for now
        let roll_output =
            (self.roll_integral + roll_rate_error * fw_p_roll + roll_rate_setpoint * fw_ff_roll)
                / 500.0;
        let pitch_output = (self.pitch_integral
            + pitch_rate_error * fw_p_pitch
            + pitch_rate_setpoint * fw_ff_pitch)
            / 500.0;

        ControllerOutput {
            roll: roll_output.min(1.0).max(-1.0),
            pitch: -pitch_output.min(1.0).max(-1.0),
        }
    }
}
