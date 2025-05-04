use crate::{events::events::{Event, StepData}, mav_crater::ComponentId};

pub trait Component {
    fn id(&self) -> ComponentId;

    fn handle_event(&mut self, event: Event);

    fn step(&mut self, step: &StepData);
} 