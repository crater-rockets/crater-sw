use core::sync::atomic::AtomicBool;

use crate::mav_crater::ComponentId;

use super::events::Event;
use alloc::sync::Arc;
use heapless::mpmc::MpMcQueue;

static QUEUE_SIZE: usize = 64;

pub struct EventItem {
    pub component_id: ComponentId,
    pub event: Event,
}

pub struct EventQueue {
    dispatcher: Arc<EventQueueInner>,
}

#[derive(Default)]
struct EventQueueInner {
    ev_queue: MpMcQueue<EventItem, QUEUE_SIZE>,
    queue_full_signal: AtomicBool,
}

impl EventQueue {
    pub fn new() -> Self {
        EventQueue {
            dispatcher: Arc::new(EventQueueInner::default()),
        }
    }

    pub fn get_publisher(&self) -> EventPublisher {
        EventPublisher {
            dispatcher: self.dispatcher.clone(),
        }
    }

    pub fn pop_event(&self) -> Option<EventItem> {
        self.dispatcher.ev_queue.dequeue()
    }

    pub fn queue_full_signaled(&self) -> bool {
        self.dispatcher
            .queue_full_signal
            .load(core::sync::atomic::Ordering::SeqCst)
    }

    pub fn clear_queue_full_signal(&self) {
        self.dispatcher
            .queue_full_signal
            .store(false, core::sync::atomic::Ordering::SeqCst);
    }
}

pub struct EventPublisher {
    dispatcher: Arc<EventQueueInner>,
}

impl EventPublisher {
    pub fn publish(&self, component_id: ComponentId, event: Event) {
        if self
            .dispatcher
            .ev_queue
            .enqueue(EventItem {
                component_id,
                event,
            })
            .is_err()
        {
            // Signal that a publisher found the queue full
            self.dispatcher
                .queue_full_signal
                .store(true, core::sync::atomic::Ordering::SeqCst);
        }
    }
}
