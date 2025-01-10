use crate::common::RcState;

#[derive(Eq, PartialEq)]
pub enum ThreePositionSwitch {
    Low,
    Middle,
    High,
}

impl ThreePositionSwitch {
    pub fn from(us: u16) -> Self {
        if us < 1400 {
            ThreePositionSwitch::Low
        } else if us < 1600 {
            ThreePositionSwitch::Middle
        } else {
            ThreePositionSwitch::High
        }
    }
}

pub struct RcCommand {
    pub roll: f32,
    pub pitch: f32,
    pub throttle: f32,
    pub yaw: f32,
    pub mode_switch: ThreePositionSwitch,
    pub pitch_offset: f32,
    pub ahrs_reset_switch: bool,
}

impl Default for RcCommand {
    fn default() -> Self {
        RcCommand {
            roll: 0.0,
            pitch: 0.0,
            throttle: 0.0,
            yaw: 0.0,
            mode_switch: ThreePositionSwitch::Low,
            pitch_offset: 0.0,
            ahrs_reset_switch: false,
        }
    }
}

pub fn radio_mapping(rc_state: &RcState, armed: &mut bool) -> RcCommand {
    let roll_us = rc_state[0];
    let pitch_us = rc_state[1];
    let throttle_us = rc_state[2];
    let yaw_us = rc_state[3];
    let armed_us = rc_state[4];
    let mode_us = rc_state[5];

    // These are some custom mappings for development
    let reset_channel_us = rc_state[7];
    let pitch_offset_us = rc_state[9];

    let roll = ((roll_us as i16 - 1500) as f32 / 500.0).max(-1.0).min(1.0);
    let pitch = ((pitch_us as i16 - 1500) as f32 / 500.0).max(-1.0).min(1.0);
    let throttle = ((throttle_us as i16 - 1000) as f32 / 1000.0)
        .max(0.0)
        .min(1.0);
    let yaw = ((yaw_us as i16 - 1500) as f32 / 500.0).max(-1.0).min(1.0);
    let pitch_offset = ((pitch_offset_us as i16 - 1500) as f32 / 500.0)
        .max(-1.0)
        .min(1.0);

    // Only arm when throttle is zero
    let new_armed_state = armed_us > 1500 && (*armed || (!*armed && throttle_us <= 1000));
    *armed = new_armed_state;

    RcCommand {
        roll,
        pitch,
        throttle,
        yaw,
        mode_switch: ThreePositionSwitch::from(mode_us),
        pitch_offset,
        ahrs_reset_switch: reset_channel_us > 1500,
    }
}
