use crater_gnc::mav_crater::ComponentId;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SimEvent {
    FsmTransition {
        fsm: String,
        source: String,
        target: String,
    },
    StartEngine,
}

pub type GncEventEnum = crater_gnc::events::Event;
pub type GncEvent = crater_gnc::events::GncEvent;

pub enum Event {
    Step,
    Sim(SimEvent),
    Gnc(GncEventEnum, ComponentId),
}

impl From<GncEvent> for Event {
    fn from(value: GncEvent) -> Self {
        Event::Gnc(value.event, value.src)
    }
}
