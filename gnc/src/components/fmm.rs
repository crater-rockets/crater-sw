use alloc::boxed::Box;
use statig::prelude::*;

use crate::{
    component::{Component, LoopContext},
    datatypes::pin::{DigitalInputState, DigitalState},
    events::{Event, EventPublisher},
    hal::channel::Receiver,
    mav_crater::ComponentId,
};

pub struct FmmHarness {
    pub rx_liftoff_pin: Box<dyn Receiver<DigitalInputState> + Send>,
}

pub struct FlightModeManager {
    state_machine: StateMachine<FMMStateMachine>,
}

impl FlightModeManager {
    pub fn new(harness: FmmHarness, event_pub: EventPublisher) -> Self {
        let state_machine = FMMStateMachine { harness, event_pub }.state_machine();

        Self { state_machine }
    }
}

impl Component for FlightModeManager {
    fn id(&self) -> ComponentId {
        ComponentId::FlightModeManager
    }

    fn handle_event(&mut self, event: Event, context: &mut LoopContext) {
        self.state_machine.handle_with_context(&event, context);
    }

    fn step(&mut self, context: &mut LoopContext) {
        self.state_machine
            .handle_with_context(&Event::Step, context);
    }
}

struct FMMStateMachine {
    harness: FmmHarness,
    event_pub: EventPublisher,
}

#[state_machine(
    initial = "State::boot()",
    state(derive(Debug)),
    superstate(derive(Debug))
)]
impl FMMStateMachine {
    #[superstate]
    #[allow(unused)]
    fn on_ground(context: &mut LoopContext, event: &Event) -> Response<State> {
        match event {
            _ => Super,
        }
    }

    #[state(superstate = "on_ground")]
    fn boot(&mut self, event: &Event) -> Response<State> {
        match event {
            Event::CmdFmmCalibrate => Transition(State::calibrating()),
            _ => Super,
        }
    }

    #[action]
    fn enter_calibrating(&self, context: &mut LoopContext) {
        self.event_pub
            .publish(Event::CmdAdaCalibrate, context.step().step_time);
    }

    #[state(superstate = "on_ground", entry_action = "enter_calibrating")]
    fn calibrating(&mut self, event: &Event) -> Response<State> {
        match event {
            Event::AdaCalibrationDone => Transition(State::ready()),
            _ => Super,
        }
    }

    #[action]
    fn enter_ready(&self, context: &mut LoopContext) {
        self.event_pub
            .publish(Event::FlightStateReady, context.step().step_time);
    }

    #[state(superstate = "on_ground", entry_action = "enter_ready")]
    fn ready(&mut self, event: &Event) -> Response<State> {
        match event {
            Event::CmdFmmArm => Transition(State::armed()),
            _ => Super,
        }
    }

    #[state(superstate = "on_ground")]
    fn armed(&mut self, event: &Event) -> Response<State> {
        match event {
            Event::Step => {
                // TODO: Avoid spurious state changes
                if let Some(lo_pin) = self.harness.rx_liftoff_pin.try_recv_last() {
                    if lo_pin.v.0 == DigitalState::Low {
                        return Transition(State::powered_ascent());
                    }
                }

                Handled
            }
            Event::CmdFmmForceLiftoff => Transition(State::powered_ascent()),
            _ => Super,
        }
    }

    #[superstate]
    fn in_flight(event: &Event) -> Response<State> {
        match event {
            _ => Super,
        }
    }

    #[action]
    fn enter_powered_ascent(&self, context: &mut LoopContext) {
        self.event_pub
            .publish(Event::FlightLiftoff, context.step().step_time);
    }

    #[state(superstate = "in_flight", entry_action = "enter_powered_ascent")]
    fn powered_ascent(event: &Event) -> Response<State> {
        match event {
            _ => Super,
        }
    }
}
