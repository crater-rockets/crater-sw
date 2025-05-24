use crate::mav_crater;

pub struct MavlinkDispatcherHarness {}

pub struct CraterMavlinkDispatcher {
    harness: MavlinkDispatcherHarness,
}

impl CraterMavlinkDispatcher {
    pub fn new(harness: MavlinkDispatcherHarness) -> Self {
        Self { harness }
    }

    pub fn dispatch(msg: mav_crater::MavMessage) {
        match msg {
            _ => {}
        }
    }
}

