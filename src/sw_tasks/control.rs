use crate::common::{AhrsState, OutputCommand, RcState};
use crate::vehicle::attitude_control::AttitudeController;
use crate::vehicle::control_logic::OutputCalculationState;
use crate::vehicle::modes::{update_mode, Mode};
use core::sync::atomic::Ordering::SeqCst;

pub(crate) async fn control_task(
    cx: crate::app::control_task::Context<'_>,
    mut ahrs_state_receiver: rtic_sync::channel::Receiver<'static, AhrsState, 1>,
    mut rc_state_receiver: rtic_sync::channel::Receiver<'static, RcState, 1>,
    mut pwm_output_sender: rtic_sync::channel::Sender<'static, OutputCommand, 1>,
) {
    use crate::app::Mono;
    use futures::{select_biased, FutureExt};
    use rtic_monotonics::Monotonic;

    let mut mode = Mode::Stabilized;

    let mut rc_timestamp = Mono::now();
    let mut rc_state = RcState {
        armed: false,
        roll: 1500,
        pitch: 1500,
        throttle: 900,
        yaw: 1500,
        mode: 1000,
        pitch_offset: 1000,
    };

    let mut ahrs_state = AhrsState {
        angles: dcmimu::EulerAngles::default(),
        rates: (0.0, 0.0, 0.0),
        _acceleration: (0.0, 0.0, 0.0),
    };

    let controller = AttitudeController::new();
    let mut previous_output_calculation_state = OutputCalculationState { controller };
    let mut temporary_output_calculation_state = previous_output_calculation_state;

    defmt::info!("control task started");
    loop {
        enum ControlTaskEvent {
            RcState(RcState),
            AhrsState(AhrsState),
        }

        let event = select_biased! {
            v = rc_state_receiver.recv().fuse() => {
                match v {
                    Ok(v) => ControlTaskEvent::RcState(v),
                    Err(_) => continue,
                }
            }
            v = ahrs_state_receiver.recv().fuse() => {
                match v {
                    Ok(v) => ControlTaskEvent::AhrsState(v),
                    Err(_) => continue,
                }
            }
        };

        match event {
            ControlTaskEvent::RcState(new_rc_state) => {
                rc_state = new_rc_state;
                rc_timestamp = Mono::now();
                update_mode(&mut mode, &rc_state);
                // Might want to switch to manual mode if AHRS update times out (without throttle?)
            }
            ControlTaskEvent::AhrsState(new_ahrs_state) => {
                ahrs_state = new_ahrs_state;
                // On AHRS sample, propagate the output state
                previous_output_calculation_state = temporary_output_calculation_state;

                if let Some(dt) = Mono::now().checked_duration_since(rc_timestamp) {
                    if rc_state.armed && dt.to_millis() > 500 {
                        mode = Mode::Failsafe;
                    }
                }
            }
        }

        if cx.shared.flags.reset_ahrs.load(SeqCst) {
            // Don't really like this yet
            previous_output_calculation_state.controller.reset();
        }

        let (output_command, output_state) = crate::vehicle::control_logic::calculate_output(
            &ahrs_state,
            &rc_state,
            &mode,
            previous_output_calculation_state,
        )
        .await;
        pwm_output_sender.try_send(output_command).ok();

        // save the state
        temporary_output_calculation_state = output_state;
    }
}
