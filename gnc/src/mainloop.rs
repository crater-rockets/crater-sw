use crate::mav_crater::MavMessage;


#[cfg(feature = "std")]
extern  crate std;

// use log::{info, error, warn, debug};

use defmt_or_log::info;
struct MainLoopInputHarness {
    // rx_mavlink: Receiver<MavMessage>,
}

pub struct MainLoop {

}

impl MainLoop {
    pub fn new() -> Self {
        MainLoop {}
    }

    pub fn run(&mut self) -> i32 {
        info!("Main loop started");

        123
    }
} 