use alloc::boxed::Box;
use thiserror::Error;

use crate::{
    DurationU64,
    component::StepData,
    component_loop::{ComponentLoop, ComponentLoopBuilder, ComponentLoopBuilderError},
    components::{
        ada::{AdaComponent, AdaHarness},
        fmm::{FlightModeManager, FmmHarness},
        navigation::{NavigationComponent, NavigationHarness},
    },
    events::{EventItem, EventQueue},
    hal::channel::Sender,
    mav_crater::ComponentId,
};

const NUM_COMPONENTS: usize = 3;

#[derive(Debug, Error, Clone)]
pub enum CraterLoopError {
    #[error("Component loop error: {0:?}")]
    ComponentBuilder(#[from] ComponentLoopBuilderError),
}

pub struct CraterLoopHarness {
    pub tx_events: Box<dyn Sender<EventItem> + Send>,
    pub fmm: FmmHarness,
    pub ada: AdaHarness,
    pub nav: NavigationHarness,
}

pub struct CraterLoop {
    component_loop: ComponentLoop<NUM_COMPONENTS>,
}

impl CraterLoop {
    pub fn new(
        event_queue: EventQueue,
        harness: CraterLoopHarness,
    ) -> Result<Self, CraterLoopError> {
        let mut loop_builder = ComponentLoopBuilder::<NUM_COMPONENTS>::new();

        let fmm = FlightModeManager::new(
            harness.fmm,
            event_queue.get_publisher(ComponentId::FlightModeManager),
        );
        loop_builder.add_component(fmm)?;

        let ada = AdaComponent::new(
            harness.ada,
            event_queue.get_publisher(ComponentId::ApogeeDetectionAlgorithm),
            DurationU64::secs(5).into(),
        );
        loop_builder.add_component(ada)?;

        let nav = NavigationComponent::new(harness.nav);
        loop_builder.add_component(nav)?;

        Ok(CraterLoop {
            component_loop: loop_builder.build(event_queue, harness.tx_events),
        })
    }

    pub fn step(&mut self, step: &StepData) {
        self.component_loop.step(step);
    }
}
