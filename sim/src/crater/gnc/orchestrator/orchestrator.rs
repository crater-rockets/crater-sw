use anyhow::Result;
use chrono::TimeDelta;
use crater_gnc::{events::GncEvent, mav_crater::ComponentId};
use statig::prelude::*;
use strum::AsRefStr;

use crate::{
    core::time::{Clock, Timestamp},
    crater::{
        channels,
        events::{Event, GncEventEnum, SimEvent},
    },
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryReceiver, TelemetrySender},
};

pub struct Orchestrator {
    rx_gnc_event: TelemetryReceiver<crater_gnc::events::GncEvent>,
    fsm: StateMachine<OrchestratorFsm>,
}

impl Orchestrator {
    pub fn new(ctx: NodeContext) -> Result<Self> {
        let fsm = OrchestratorFsm {
            tx_sim_event: ctx.telemetry().publish_mp(channels::sim::SIM_EVENTS)?,
            tx_gnc_event: ctx.telemetry().publish_mp(channels::gnc::GNC_EVENTS)?,
        }
        .state_machine();

        Ok(Self {
            rx_gnc_event: ctx.telemetry().subscribe_mp(
                channels::gnc::GNC_EVENTS,
                crate::utils::capacity::Capacity::Unbounded,
            )?,
            fsm,
        })
    }
}

impl Node for Orchestrator {
    fn step(&mut self, _i: usize, _dt: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
        let mut step_ctx = StepContext {
            time: Timestamp::now(clock),
        };

        while let Ok(ev) = self.rx_gnc_event.try_recv() {
            self.fsm.handle_with_context(&ev.1.into(), &mut step_ctx);
        }

        self.fsm.handle_with_context(&Event::Step, &mut step_ctx);

        Ok(StepResult::Continue)
    }
}

pub struct StepContext {
    time: Timestamp,
}

pub struct OrchestratorFsm {
    tx_gnc_event: TelemetrySender<GncEvent>,
    tx_sim_event: TelemetrySender<SimEvent>,
}

#[state_machine(
    initial = "State::init()",
    state(derive(Debug, Clone, AsRefStr)),
    superstate(derive(Debug)),
    after_transition = "Self::after_transition"
)]
impl OrchestratorFsm {
    #[state]
    fn init(&mut self, context: &mut StepContext, event: &Event) -> Response<State> {
        match event {
            Event::Step => {
                self.tx_gnc_event.send(
                    context.time,
                    GncEvent {
                        src: ComponentId::Ground,
                        event: GncEventEnum::CmdFmmCalibrate,
                    },
                );
                Transition(State::wait_ready())
            }
            _ => Super,
        }
    }

    #[state]
    fn wait_ready(context: &mut StepContext, event: &Event) -> Response<State> {
        match event {
            Event::Gnc(GncEventEnum::FlightStateReady, _) => Transition(State::arm(context.time)),
            Event::Step => Handled,
            _ => Super,
        }
    }

    #[action]
    fn enter_arm(&mut self, context: &mut StepContext) {
        self.tx_gnc_event.send(
            context.time,
            GncEvent {
                src: ComponentId::Ground,
                event: crater_gnc::events::Event::CmdFmmArm,
            },
        );
    }

    #[state(entry_action = "enter_arm")]
    fn arm(
        &mut self,
        entry_time: &mut Timestamp,
        context: &mut StepContext,
        event: &Event,
    ) -> Response<State> {
        match event {
            Event::Step => {
                if context.time.monotonic - entry_time.monotonic > TimeDelta::seconds(1) {
                    self.tx_sim_event.send(context.time, SimEvent::StartEngine);
                    Transition(State::flying(context.time))
                } else {
                    Handled
                }
            }
            _ => Super,
        }
    }

    #[state]
    fn flying(
        entry_time: &mut Timestamp,
        context: &mut StepContext,
        event: &Event,
    ) -> Response<State> {
        match event {
            Event::Step => {
                if context.time.monotonic - entry_time.monotonic > TimeDelta::seconds(2) {}

                Handled
            }
            _ => Super,
        }
    }
}

impl OrchestratorFsm {
    fn after_transition(&mut self, source: &State, target: &State, context: &mut StepContext) {
        self.tx_sim_event.send(
            context.time,
            SimEvent::FsmTransition {
                fsm: "orchestrator".to_string(),
                source: source.as_ref().to_string(),
                target: target.as_ref().to_string(),
            },
        );
    }
}
