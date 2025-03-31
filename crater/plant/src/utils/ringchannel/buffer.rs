use std::num::NonZero;

use ringbuffer::{AllocRingBuffer, GrowableAllocRingBuffer, RingBuffer};

#[derive(Debug, Clone)]
enum BufType<T> {
    Bounded(ringbuffer::AllocRingBuffer<T>),
    Unbounded(ringbuffer::GrowableAllocRingBuffer<T>),
}

#[derive(Debug, Clone)]
pub struct Buffer<T> {
    buf: BufType<T>,
}

impl<T> Buffer<T> {
    pub fn bounded(capacity: NonZero<usize>) -> Self {
        Self {
            buf: BufType::Bounded(AllocRingBuffer::new(capacity.get())),
        }
    }

    pub fn unbounded() -> Self {
        Self {
            buf: BufType::Unbounded(GrowableAllocRingBuffer::new()),
        }
    }

    pub fn push(&mut self, value: T) {
        match &mut self.buf {
            BufType::Bounded(b) => b.push(value),
            BufType::Unbounded(b) => b.push(value),
        }
    }

    pub fn dequeue(&mut self) -> Option<T> {
        match &mut self.buf {
            BufType::Bounded(b) => b.dequeue(),
            BufType::Unbounded(b) => b.dequeue(),
        }
    }

    pub fn len(&self) -> usize {
        match &self.buf {
            BufType::Bounded(b) => b.len(),
            BufType::Unbounded(b) => b.len(),
        }
    }

    pub fn is_empty(&self) -> bool {
        match &self.buf {
            BufType::Bounded(b) => b.is_empty(),
            BufType::Unbounded(b) => b.is_empty(),
        }
    }
}

