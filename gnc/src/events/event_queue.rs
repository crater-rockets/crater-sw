use core::sync::atomic::AtomicBool;

use crate::{Instant, common::Ts, mav_crater::ComponentId};

use super::event::Event;
use alloc::sync::Arc;
use heapless::mpmc::MpMcQueue;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

static QUEUE_SIZE: usize = 64;

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct GncEvent {
    pub src: ComponentId,
    pub event: Event,
}

#[derive(Default)]
pub struct EventQueue {
    dispatcher: Arc<EventQueueInner>,
}

#[derive(Default)]
struct EventQueueInner {
    ev_queue: MpMcQueue<Ts<GncEvent>, QUEUE_SIZE>,
    queue_full_signal: AtomicBool,
}

impl EventQueue {
    pub fn new() -> Self {
        EventQueue {
            dispatcher: Arc::new(EventQueueInner::default()),
        }
    }

    pub fn get_publisher(&self, src: ComponentId) -> EventPublisher {
        EventPublisher {
            dispatcher: self.dispatcher.clone(),
            src,
        }
    }

    pub fn pop_event(&self) -> Option<Ts<GncEvent>> {
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
    src: ComponentId,
}

impl EventPublisher {
    pub fn publish(&self, event: Event, ts: Instant) {
        if self
            .dispatcher
            .ev_queue
            .enqueue(Ts {
                t: ts,
                v: GncEvent {
                    src: self.src,
                    event,
                },
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
