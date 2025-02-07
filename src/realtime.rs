use std::path::Path;

use crate::RobotResult;

pub trait RealtimeBehavior<C, S, H> {
    fn enter_realtime(&mut self, realtime_config: C) -> RobotResult<H>;
    fn exit_realtime(&mut self) -> RobotResult<()>;
    fn read_state(&self) -> S;
    fn quality_of_service(&self) -> f64;
    fn is_hardware_realtime(&self) -> bool {
        if cfg!(target_os = "linux") {
            Path::new("/sys/kernel/realtime").exists()
        } else if cfg!(target_os = "windows") {
            true
        } else {
            println!("Unknown OS, assuming realtime kernel.");
            true
        }
    }
}
