use crate::common::RcState;

enum ThreePositionSwitch {
    Low,
    Middle,
    High,
}

enum TwoPositionSwitch {
    Low,
    High,
}

pub struct RadioMapping {
    pub roll: f32,
    pub pitch: f32,
    pub throttle: f32,
    pub yaw: f32,
    pub armed: TwoPositionSwitch,
    pub mode_switch: ThreePositionSwitch,
    pub pitch_offset: f32,
}

pub fn radio_mapping(rc_state: &RcState) -> RadioMapping {
    RadioMapping {
        roll: rc_state.roll,
        pitch: rc_state.pitch,
        throttle: rc_state.throttle,
        yaw: rc_state.yaw,
        armed: if rc_state.armed {
            TwoPositionSwitch::High
        } else {
            TwoPositionSwitch::Low
        },
        mode_switch: if rc_state.mode < 0.33 {
            ThreePositionSwitch::Low
        } else if rc_state.mode < 0.66 {
            ThreePositionSwitch::Middle
        } else {
            ThreePositionSwitch::High
        },
        pitch_offset: rc_state.pitch_offset,
    }
}
