use crate::{
    Duration, Instant,
    common::Ts,
    component::{Component, LoopContext},
    datatypes::sensors::PressureSensorSample,
    events::{Event, EventPublisher},
    hal::channel::{Receiver, Sender},
    mav_crater::ComponentId,
};
use alloc::boxed::Box;
use statig::prelude::*;

pub struct AdaHarness {
    pub rx_static_pressure: Box<dyn Receiver<PressureSensorSample> + Send>,

    pub tx_ada_data: Box<dyn Sender<AdaResult> + Send>,
}

pub struct AdaComponent {
    state_machine: StateMachine<AdaStateMachine>,
}

impl AdaComponent {
    pub fn new(
        harness: AdaHarness,
        event_pub: EventPublisher,
        shadow_mode_timeout: Duration,
    ) -> Self {
        let state_machine = AdaStateMachine {
            harness,
            event_pub,
            shadow_mode_timeout,
            ada_algo: AdaAlgorithm::default(),
        }
        .state_machine();

        Self { state_machine }
    }
}
impl Component for AdaComponent {
    fn id(&self) -> ComponentId {
        ComponentId::ApogeeDetectionAlgorithm
    }

    fn handle_event(&mut self, event: Event, context: &mut LoopContext) {
        self.state_machine.handle_with_context(&event, context);
    }

    fn step(&mut self, context: &mut LoopContext) {
        self.state_machine
            .handle_with_context(&Event::Step, context);
    }
}

struct AdaStateMachine {
    harness: AdaHarness,
    event_pub: EventPublisher,
    shadow_mode_timeout: Duration,

    ada_algo: AdaAlgorithm,
}

#[state_machine(initial = "State::idle()")]
impl AdaStateMachine {
    #[state]
    fn idle(event: &Event, context: &mut LoopContext) -> Response<State> {
        match event {
            Event::CmdAdaCalibrate => Transition(State::calibrating(
                context.step().step_time,
                AdaCalibration::default(),
            )),
            _ => Super,
        }
    }

    #[state]
    fn calibrating(
        &mut self,
        entry_time: &mut Instant,
        calib: &mut AdaCalibration,
        context: &mut LoopContext,
        event: &Event,
    ) -> Response<State> {
        match event {
            Event::Step => {
                if let Some(press) = self.harness.rx_static_pressure.try_recv() {
                    calib.ref_pressure_pa = press.v.pressure_pa;
                }

                if context.step().step_time.0 - entry_time.0 >= self.shadow_mode_timeout.0 {
                    self.event_pub
                        .publish(Event::AdaCalibrationDone, context.step().step_time);
                    self.ada_algo.update_calib(calib.clone());
                    Transition(State::ready())
                } else {
                    Handled
                }
            }
            _ => Super,
        }
    }

    #[state]
    fn ready(context: &mut LoopContext, event: &Event) -> Response<State> {
        match event {
            Event::FlightLiftoff => Transition(State::shadow_mode(context.step().step_time)),
            _ => Super,
        }
    }

    #[state]
    fn shadow_mode(
        &mut self,
        entry_time: &mut Instant,
        context: &mut LoopContext,
        event: &Event,
    ) -> Response<State> {
        match event {
            Event::Step => {
                self.update_ada();

                if context.step().step_time.0 - entry_time.0 >= self.shadow_mode_timeout.0 {
                    Transition(State::active())
                } else {
                    Handled
                }
            }
            _ => Super,
        }
    }

    #[state]
    fn active(&mut self, event: &Event) -> Response<State> {
        match event {
            Event::Step => {
                self.update_ada();

                Handled
            }
            _ => Super,
        }
    }

    fn update_ada(&mut self) {
        if let Some(press) = self.harness.rx_static_pressure.try_recv() {
            let out = self.ada_algo.update(press);

            let _ = self.harness.tx_ada_data.try_send(out.t, out.v);
        }
    }
}

#[derive(Debug, Clone)]
pub struct AdaCalibration {
    ref_pressure_pa: f32,
}

impl Default for AdaCalibration {
    fn default() -> Self {
        AdaCalibration {
            ref_pressure_pa: 101325.0f32,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct AdaAlgorithm {
    calib: AdaCalibration,
}

#[derive(Debug, Clone)]
pub struct AdaResult {
    pub altitude_m: f32,
    pub vertical_speed_m_s: f32,
}

impl AdaAlgorithm {
    fn update_calib(&mut self, calib: AdaCalibration) {
        self.calib = calib;
    }

    /// Just a mockup for now
    fn update(&mut self, press: Ts<PressureSensorSample>) -> Ts<AdaResult> {
        let v = AdaResult {
            altitude_m: (press.v.pressure_pa - self.calib.ref_pressure_pa) / 2f32,
            vertical_speed_m_s: -press.v.pressure_pa / 100f32,
        };

        Ts::new(press.t, v)
    }
}
