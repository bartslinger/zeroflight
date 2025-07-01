use embassy_time::Timer;

#[embassy_executor::task]
pub async fn run_fast_io_crsf_task() {
    loop {
        Timer::after_secs(1).await;
    }
}
