use crate::component::Component;
use crate::events::event_queue::EventQueue;
use crate::events::events::StepData;
use crate::mav_crater::ComponentId;
use alloc::boxed::Box;
use heapless::Vec;

pub struct ComponentLoop<const N: usize> {
    event_queue: EventQueue,
    components: Vec<Box<dyn Component>, N>,
}

impl<const N: usize> ComponentLoop<N> {
    pub fn step(&mut self, step: &StepData) {
        while let Some(event) = self.event_queue.pop_event() {
            for component in &mut self.components {
                if event.component_id == ComponentId::All || event.component_id == component.id() {
                    component.handle_event(event.event.clone());
                }
            }
        }

        for component in &mut self.components {
            component.step(step);
        }
    }
}

pub enum ComponentLoopBuilderError {
    TooManyComponents,
}

pub struct ComponentLoopBuilder<const N: usize> {
    components: Vec<Box<dyn Component>, N>,
}

impl<const N: usize> ComponentLoopBuilder<N> {
    pub fn new() -> Self {
        ComponentLoopBuilder {
            components: Vec::new(),
        }
    }

    pub fn add_component<T>(&mut self, component: T) -> Result<(), ComponentLoopBuilderError>
    where
        T: Component + 'static,
    {
        if self.components.push(Box::new(component)).is_ok() {
            Ok(())
        } else {
            Err(ComponentLoopBuilderError::TooManyComponents)
        }
    }

    pub fn build(self, event_queue: EventQueue) -> ComponentLoop<N> {
        ComponentLoop {
            event_queue,
            components: self.components,
        }
    }
}
