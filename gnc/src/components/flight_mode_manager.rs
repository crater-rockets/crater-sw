use statig::prelude::*;

use crate::{
    component::Component,
    events::events::{Event, StepData},
    mav_crater::ComponentId,
};

pub struct FlightModeManager {
    state_machine: StateMachine<FMMStateMachine>,
}

impl FlightModeManager {
    pub fn new() -> Self {
        let state_machine = FMMStateMachine.state_machine();

        Self { state_machine }
    }
}

impl Component for FlightModeManager {
    fn id(&self) -> ComponentId {
        ComponentId::FlightModeManager
    }

    fn handle_event(&mut self, event: Event) {
        self.state_machine.handle(&event);
    }

    fn step(&mut self, step: &StepData) {
        self.state_machine.handle(&Event::Step(*step));
    }
}

struct FMMStateMachine;

#[state_machine(initial = "State::armed()")]
impl FMMStateMachine {
    #[superstate]
    fn on_ground(event: &Event) -> Response<State> {
        match event {
            _ => Super,
        }
    }

    #[state(superstate = "on_ground")]
    fn armed(event: &Event) -> Response<State> {
        match event {
            _ => Super,
        }
    }

    #[superstate]
    fn in_flight(event: &Event) -> Response<State> {
        match event {
            _ => Super,
        }
    }

    #[state(superstate = "in_flight")]
    fn powered_ascent(entry_time: &mut f64, event: &Event) -> Response<State> {
        match event {
            _ => Super,
        }
    }
}
