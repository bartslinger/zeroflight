use crate::common::{ActuatorCommands, OutputCommand};

pub fn output_mixing(command: &OutputCommand) -> ActuatorCommands {
    let s1 = if command.armed {
        command.throttle.max(0.0).min(1.0)
    } else {
        0.0
    };
    let s2 = 0.0;

    // scale roll 60%
    let roll_middle = -0.092; //1454; slight offset on the t250
    let roll_min = roll_middle - 0.6; //- 300;
    let roll_max = roll_middle + 0.6; //+ 300;
    let s3 = (command.roll * -0.6 + roll_middle)
        .max(roll_min)
        .min(roll_max);
    let s4 = (command.roll * -0.6 + roll_middle)
        .max(roll_min)
        .min(roll_max);
    // let s3 = (((command.roll as i16 - 1500) * 6 / 10 * -1 + roll_middle) as u16)
    //     .min(roll_max as u16)
    //     .max(roll_min as u16);
    // let s4 = (((command.roll as i16 - 1500) * 6 / 10 * -1 + roll_middle) as u16)
    //     .min(roll_max as u16)
    //     .max(roll_min as u16);

    let s5 = (command.pitch * -1.0 + command.yaw * -0.4)
        .max(-1.0)
        .min(1.0);
    let s6 = (command.pitch * 1.0 + command.yaw * -0.4)
        .max(-1.0)
        .min(1.0);
    // let s5 = ((command.pitch as i16 - 1500) * -1 + 1500) as u16;
    // let s6 = command.pitch;

    ActuatorCommands {
        s1: ((s1 * 1000.0) as i16 + 1000) as u16,
        s2: ((s2 * 1000.0) as i16 + 1000) as u16,
        s3: ((s3 * 500.0) as i16 + 1500) as u16,
        s4: ((s4 * 500.0) as i16 + 1500) as u16,
        s5: ((s5 * 500.0) as i16 + 1500) as u16,
        s6: ((s6 * 500.0) as i16 + 1500) as u16,
    }
}
