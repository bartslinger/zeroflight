use crate::common::{ActuatorPwmCommands, AhrsState, ImuData, MaybeUpdatedValue, RcState};
use crate::vehicle::{main_loop, MainState};
use crate::IMUDATAPOOL;
use heapless::pool::boxed::Box;

pub(crate) async fn control_task(
    cx: crate::app::control_task::Context<'_>,
    mut imu_data_receiver: rtic_sync::channel::Receiver<'static, Box<IMUDATAPOOL>, 1>,
    mut imu_to_ahrs_data_sender: rtic_sync::channel::Sender<'static, ImuData, 1>,
    mut ahrs_state_rx: rtic_sync::channel::Receiver<'static, AhrsState, 1>,
    mut rc_state_receiver: rtic_sync::channel::Receiver<'static, RcState, 1>,
    mut pwm_output_sender: rtic_sync::channel::Sender<'static, ActuatorPwmCommands, 1>,
) {
    let dwt = cx.local.dwt;
    use futures::{select_biased, FutureExt};

    let mut rc_state = MaybeUpdatedValue::new([0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);

    let mut imu_data = MaybeUpdatedValue::new(ImuData {
        acceleration: (0.0, 0.0, 0.0),
        rates: (0.0, 0.0, 0.0),
    });
    let mut ahrs_state = MaybeUpdatedValue::new(AhrsState::default());

    let mut main_loop_state = MainState::default();
    let mut imu_forward_counter: u32 = 0;

    let mut main_loop_counter: u32 = 0;
    let mut previous_main_loop_tick = None;
    let mut average_loop_delta: f32 = 1_000.0;
    let mut min_loop_delta: u32 = u32::MAX;
    let mut max_loop_delta: u32 = 0;

    let mut average_runtime: f32 = 10.0;
    let mut min_runtime: u32 = u32::MAX;
    let mut max_runtime: u32 = 0;

    defmt::info!("control task started");
    loop {
        // Main reason for using this enum is because select_biased! doesn't work with code
        // formatting
        enum ControlTaskEvent {
            ImuData(Box<IMUDATAPOOL>),
            RcState(RcState),
            AhrsState(AhrsState),
        }

        let event = select_biased! {
            v = imu_data_receiver.recv().fuse() => {
                match v {
                    Ok(v) => ControlTaskEvent::ImuData(v),
                    Err(_) => continue,
                }
            }
            v = rc_state_receiver.recv().fuse() => {
                match v {
                    Ok(v) => ControlTaskEvent::RcState(v),
                    Err(_) => continue,
                }
            }
            v = ahrs_state_rx.recv().fuse() => {
                match v {
                    Ok(v) => ControlTaskEvent::AhrsState(v),
                    Err(_) => continue,
                }
            }
        };

        match event {
            ControlTaskEvent::ImuData(new_imu_data) => {
                let new_imu_data = *new_imu_data;
                imu_data.update(new_imu_data);

                // Forward every 4th IMU data to the AHRS task
                imu_forward_counter += 1;
                if imu_forward_counter % 4 == 0 {
                    imu_forward_counter = 0;
                    if let Err(_) = imu_to_ahrs_data_sender.try_send(new_imu_data) {
                        defmt::error!("error sending imu data to ahrs");
                    }
                }
            }
            ControlTaskEvent::RcState(new_rc_state) => {
                rc_state.update(new_rc_state);
            }
            ControlTaskEvent::AhrsState(new_ahrs_state) => {
                ahrs_state.update(new_ahrs_state);
            } // TODO: Timeout? if no imu data, map rc directly to output?
        }

        if let Some(timestamped_imu_data) = imu_data.updated() {
            // Calculate loop delta for debugging
            let tick = dwt.cyccnt.read();
            if let Some(previous_tick) = previous_main_loop_tick {
                let delta = tick.wrapping_sub(previous_tick);
                min_loop_delta = delta.min(min_loop_delta);
                max_loop_delta = delta.max(max_loop_delta);
                average_loop_delta = 0.99 * average_loop_delta + 0.01 * delta as f32;
            }
            if main_loop_counter % 1000 == 0 {
                main_loop_counter = 0;

                // defmt::info!(
                //     "min: {} max: {} avg: {}",
                //     min_loop_delta / 168,
                //     max_loop_delta / 168,
                //     average_loop_delta / 168.0,
                // );
                // defmt::info!(
                //     "runtime min: {} max: {} avg: {}",
                //     min_runtime / 168,
                //     max_runtime / 168,
                //     average_runtime / 168.0
                // );
                min_loop_delta = u32::MAX;
                max_loop_delta = 0;
            }

            previous_main_loop_tick = Some(tick);
            main_loop_counter += 1;

            // Run the main loop
            let output = main_loop(
                &mut main_loop_state,
                timestamped_imu_data.value(),
                &mut ahrs_state,
                &mut rc_state,
                0.001,
            );

            let after = dwt.cyccnt.read();
            let runtime = after.wrapping_sub(tick);
            min_runtime = runtime.min(min_runtime);
            max_runtime = runtime.max(max_runtime);
            average_runtime = 0.99 * average_runtime + 0.01 * runtime as f32;

            if let Err(_) = pwm_output_sender.try_send(output) {
                defmt::error!("error sending pwm output");
            }
        }
    }
}
