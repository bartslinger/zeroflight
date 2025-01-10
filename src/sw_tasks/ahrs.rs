use crate::common::{AhrsState, ImuData};
use crate::vehicle::{ahrs_loop, AhrsLoopState};

pub async fn ahrs_task(
    _cx: crate::app::ahrs_task::Context<'_>,
    mut imu_to_ahrs_data_receiver: rtic_sync::channel::Receiver<'static, ImuData, 1>,
    mut ahrs_state_tx: rtic_sync::channel::Sender<'static, AhrsState, 1>,
) {
    let mut state = AhrsLoopState::default();
    loop {
        let imu_data = match imu_to_ahrs_data_receiver.recv().await {
            Ok(v) => v,
            Err(_) => continue,
        };

        let ahrs_state = ahrs_loop(&mut state, &imu_data, 0.004);
        if let Err(_) = ahrs_state_tx.send(ahrs_state).await {
            defmt::error!("ahrs_state_tx send failed");
        }
    }
}
