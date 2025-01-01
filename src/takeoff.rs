use crate::crsf::RcState;
use crate::imu::AhrsState;

const PI: f32 = 3.14159265358979323846264338327950288_f32;

pub(crate) struct TakeoffModeCommand {
    pub(crate) throttle: u16,
    pub(crate) pitch_setpoint: f32,
}

pub(crate) enum TakeoffState {
    Inactive,        // Not doing anything
    Armed,           // Pitch up, integrator disabled, wait for throttle above threshold
    WaitingForThrow, // Running at idle throttle now
    TakingOff,       // Full throttle, integrator enabled
}
pub(crate) struct TakeoffMode {
    state: TakeoffState,
}

impl TakeoffMode {
    pub(crate) fn new() -> Self {
        Self {
            state: TakeoffState::Inactive,
        }
    }

    pub(crate) fn state(&self) -> &TakeoffState {
        &self.state
    }

    pub(crate) fn reset(&mut self) {
        self.state = TakeoffState::Inactive;
    }

    pub(crate) fn is_inactive(&self) -> bool {
        matches!(self.state, TakeoffState::Inactive)
    }

    pub(crate) fn activate(&mut self) {
        self.state = TakeoffState::Armed;
    }

    pub(crate) fn update(
        &mut self,
        rc_state: RcState,
        ahrs_state: AhrsState,
    ) -> TakeoffModeCommand {
        let mut command = TakeoffModeCommand {
            throttle: 0,
            pitch_setpoint: 30.0 * PI / 180.0,
        };
        match self.state {
            TakeoffState::Inactive => {}
            TakeoffState::Armed => {
                if rc_state.throttle > 1100 {
                    self.state = TakeoffState::WaitingForThrow;
                }
            }
            TakeoffState::WaitingForThrow => {
                command.throttle = 1100;
                if ahrs_state.acceleration.0 > 0.5 * 9.80665 {
                    self.state = TakeoffState::TakingOff;
                }
            }
            TakeoffState::TakingOff => {
                command.throttle = 2000;
            }
        }
        command
    }
}
