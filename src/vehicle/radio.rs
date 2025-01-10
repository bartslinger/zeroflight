use crate::common::RcState;

#[derive(Eq, PartialEq)]
pub enum ThreePositionSwitch {
    Low,
    Middle,
    High,
}

pub struct RadioMapping {
    pub roll: f32,
    pub pitch: f32,
    pub throttle: f32,
    pub yaw: f32,
    pub armed: bool,
    pub mode_switch: ThreePositionSwitch,
    pub pitch_offset: f32,
    pub ahrs_reset_switch: bool,
}

pub fn radio_mapping(rc_state: &RcState) -> RadioMapping {
    // TODO: change the RcState to contain the raw pwm values per channel
    RadioMapping {
        roll: rc_state.roll,
        pitch: rc_state.pitch,
        throttle: rc_state.throttle,
        yaw: rc_state.yaw,
        armed: rc_state.armed,
        mode_switch: if rc_state.mode < 0.33 {
            ThreePositionSwitch::Low
        } else if rc_state.mode < 0.66 {
            ThreePositionSwitch::Middle
        } else {
            ThreePositionSwitch::High
        },
        pitch_offset: rc_state.pitch_offset,
        ahrs_reset_switch: false,
    }
}
