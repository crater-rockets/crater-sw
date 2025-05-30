use crater_gnc::mav_crater::ComponentId;

#[derive(Debug, Clone, PartialEq)]
pub enum SimEvent {
    FsmTransition {
        fsm: String,
        source: String,
        target: String,
    },
    StartEngine,
}

pub type GncEvent = crater_gnc::events::Event;
pub type GncEventItem = crater_gnc::events::EventItem;

pub enum Event {
    Step,
    Sim(SimEvent),
    Gnc(GncEvent, ComponentId),
}

impl From<GncEventItem> for Event {
    fn from(value: GncEventItem) -> Self {
        Event::Gnc(value.event, value.src)
    }
}
