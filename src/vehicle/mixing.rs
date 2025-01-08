use crate::common::{ActuatorCommands, OutputCommand};

pub fn output_mixing(command: &OutputCommand) -> ActuatorCommands {
    let s1 = if command.armed {
        command.throttle.min(2000).max(1000)
    } else {
        900
    };
    let s2 = 900;

    // scale roll 60%
    let roll_middle = 1454;
    let roll_min = roll_middle - 300;
    let roll_max = roll_middle + 300;
    let s3 = (((command.roll as i16 - 1500) * 6 / 10 * -1 + roll_middle) as u16)
        .min(roll_max as u16)
        .max(roll_min as u16);
    let s4 = (((command.roll as i16 - 1500) * 6 / 10 * -1 + roll_middle) as u16)
        .min(roll_max as u16)
        .max(roll_min as u16);

    let s5 = ((command.pitch as i16 - 1500) * -1 + 1500) as u16;
    let s6 = command.pitch;

    ActuatorCommands {
        s1,
        s2,
        s3,
        s4,
        s5,
        s6,
    }
}
