use alloc::boxed::Box;
use thiserror::Error;

use crate::{Instant, common::Ts};

pub trait Receiver<T> {
    fn try_recv(&mut self) -> Option<Ts<T>>;

    fn try_recv_last(&mut self) -> Option<Ts<T>> {
        let mut last = None;

        while let Some(msg) = self.try_recv() {
            last = Some(msg);
        }

        last
    }

    fn len(&self) -> usize;

    fn capacity(&self) -> usize;

    fn is_empty(&self) -> bool;

    fn is_full(&self) -> bool;

    fn num_lagged(&self) -> usize;
}

#[derive(Error, Debug)]
pub struct Full<T>(pub Ts<T>);

pub trait Sender<T> {
    fn try_send(&mut self, ts: Instant, item: T) -> Result<(), Full<T>>;
    fn send_immediate(&mut self, ts: Instant, item: T);
}

#[derive(Error, Debug, Clone)]
pub enum ChannelError {
    #[error("No more senders available for this channel")]
    TooManySenders,

    #[error("No more receivers available for this channel")]
    TooManyReceivers,
}

pub trait Channel<T, const RX: usize, const TX: usize> {
    fn new_sender(&mut self) -> Result<Box<dyn Sender<T>>, ChannelError>;
    fn new_receiver(&mut self) -> Result<Box<dyn Receiver<T>>, ChannelError>;
}
