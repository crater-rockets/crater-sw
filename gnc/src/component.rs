use crate::{Duration, Instant, events::Event, mav_crater::ComponentId};

#[derive(Debug, Clone, Copy)]
pub struct StepData {
    pub step_time: Instant,
    pub step_interval: Duration,
    pub step_count: u32,
}

pub struct LoopContext {
    step: StepData,
}

impl LoopContext {
    pub fn new(step: StepData) -> Self {
        Self { step }
    }

    pub fn step(&self) -> &StepData {
        &self.step
    }
}

pub trait Component {
    fn id(&self) -> ComponentId;

    fn handle_event(&mut self, event: Event, context: &mut LoopContext);

    fn step(&mut self, context: &mut LoopContext);
}
