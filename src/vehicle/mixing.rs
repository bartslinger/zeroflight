use crate::common::{ActuatorPwmCommands, OutputCommand};

pub fn output_mixing(command: &OutputCommand) -> ActuatorPwmCommands {
    let motor_1 = if command.armed {
        command.throttle.max(0.0).min(1.0)
    } else {
        0.0
    };
    let motor_2_unused = 0.0;

    // scale roll 60%
    let roll_middle = -0.092; //1454; slight offset on the t250
    let roll_min = roll_middle - 0.6; //- 300;
    let roll_max = roll_middle + 0.6; //+ 300;
    let aileron_left = (command.roll * -0.6 + roll_middle)
        .max(roll_min)
        .min(roll_max);
    let aileron_right = aileron_left;

    let vtail_left = (command.pitch * -1.0 + command.yaw * -0.4)
        .max(-1.0)
        .min(1.0);
    let vtail_right = (command.pitch * 1.0 + command.yaw * -0.4)
        .max(-1.0)
        .min(1.0);

    ActuatorPwmCommands {
        s1: ((motor_1 * 1000.0) as i16 + 1000) as u16,
        s2: ((motor_2_unused * 1000.0) as i16 + 1000) as u16,
        s3: ((aileron_left * 500.0) as i16 + 1500) as u16,
        s4: ((aileron_right * 500.0) as i16 + 1500) as u16,
        s5: ((vtail_left * 500.0) as i16 + 1500) as u16,
        s6: ((vtail_right * 500.0) as i16 + 1500) as u16,
    }
}
