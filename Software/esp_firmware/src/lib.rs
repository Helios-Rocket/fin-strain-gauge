#![no_std]
pub mod fin_driver;
pub mod logging;
pub mod lsm;
pub mod sd;
pub mod wifi;

#[macro_export]
macro_rules! now_ms {
    () => {
        ::esp_hal::time::Instant::now()
            .duration_since_epoch()
            .as_millis()
    };
}
