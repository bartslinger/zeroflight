use crate::common::RcState;

pub enum Mode {
    Stabilized,
    Acro,
    Manual,
    Failsafe,
}

pub fn update_mode(mode: &mut Mode, rc_state: &RcState) {
    // This function gets called when a new RC state is received

    if rc_state.mode < 0.4 {
        *mode = Mode::Stabilized;
    } else if rc_state.mode >= 0.4 && rc_state.mode < 0.6 {
        *mode = Mode::Acro;
    } else {
        // manual mode
        *mode = Mode::Manual;
    }
}
