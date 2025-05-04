use crate::{Duration, Instant};

#[derive(Debug, Clone)]
pub enum Event {
    Step(StepData), // Special event to perform a step in the FSMs
    Liftoff,
    Meco,
}

#[derive(Debug, Clone, Copy)]
pub struct StepData {
    pub step_time: Instant,
    pub step_interval: Duration,
    pub step_count: u32,
}
