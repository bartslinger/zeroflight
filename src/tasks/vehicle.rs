use embassy_time::Timer;

#[embassy_executor::task]
async fn fast_vehicle_task() {
    loop {
        Timer::after_secs(1).await;
    }
}
