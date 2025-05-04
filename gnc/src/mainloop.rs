use crate::mav_crater::MavMessage;


#[cfg(feature = "std")]
extern  crate std;

#[cfg(feature = "std")]
use std::println;

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
        #[cfg(feature = "std")]
        println!("Main loop started");

        123
    }
} 