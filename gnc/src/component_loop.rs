use crate::component::{Component, LoopContext, StepData};
use crate::events::{GncEvent, EventQueue};
use crate::hal::channel::Sender;
use crate::mav_crater::ComponentId;
use alloc::boxed::Box;
use heapless::Vec;
use thiserror::Error;

pub struct ComponentLoop<const N: usize> {
    event_queue: EventQueue,
    tx_event: Box<dyn Sender<GncEvent> + Send>,
    components: Vec<Box<dyn Component + Send>, N>,
}

impl<const N: usize> ComponentLoop<N> {
    pub fn step(&mut self, step: &StepData) {
        let mut loop_context = LoopContext::new(*step);

        while let Some(event) = self.event_queue.pop_event() {
            for component in &mut self.components {
                component.handle_event(event.v.event, &mut loop_context);
            }

            if event.v.src != ComponentId::Ground {
                let _ = self.tx_event.try_send(event.t, event.v);
            }
        }

        for component in &mut self.components {
            component.step(&mut loop_context);
        }
    }
}

#[derive(Debug, Clone, Error)]
pub enum ComponentLoopBuilderError {
    #[error("No space for more components")]
    TooManyComponents,
}

pub struct ComponentLoopBuilder<const N: usize> {
    components: Vec<Box<dyn Component + Send>, N>,
}

impl<const N: usize> ComponentLoopBuilder<N> {
    pub fn new() -> Self {
        ComponentLoopBuilder {
            components: Vec::new(),
        }
    }

    pub fn add_component<T: Send>(&mut self, component: T) -> Result<(), ComponentLoopBuilderError>
    where
        T: Component + 'static,
    {
        if self.components.push(Box::new(component)).is_ok() {
            Ok(())
        } else {
            Err(ComponentLoopBuilderError::TooManyComponents)
        }
    }

    pub fn build(
        self,
        event_queue: EventQueue,
        tx_event: Box<dyn Sender<GncEvent> + Send>,
    ) -> ComponentLoop<N> {
        ComponentLoop {
            event_queue,
            tx_event,
            components: self.components,
        }
    }
}
