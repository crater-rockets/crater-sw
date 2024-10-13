use thiserror::Error;

use std::{
    num::NonZero,
    sync::{Arc, Condvar, Mutex},
};

use crate::utils::capacity::Capacity;

use super::{
    buffer::Buffer,
    select::{SelectGroup, SelectToken, Selectable},
};

#[derive(Debug, Error, PartialEq, Eq)]
pub enum ChannelError {
    #[error("The channel was closed (no sender)")]
    Closed,

    #[error("No data available in channel")]
    Empty,
}

#[derive(Debug)]
pub struct Channel<T> {
    inner: Mutex<ChannelInner<T>>,
}

#[derive(Debug)]
struct ChannelInner<T> {
    receivers: Vec<(usize, Arc<ReceiverShared<T>>)>,
    counter: usize,
    is_closed: bool,
}

impl<T: Clone> Channel<T> {
    fn write(&self, data: T) {
        let receivers = &self.inner.lock().unwrap().receivers;

        for (_, receiver) in receivers.iter() {
            receiver.write(data.clone());
        }
    }
}

impl<T> Default for Channel<T> {
    fn default() -> Self {
        Self {
            inner: Mutex::new(ChannelInner {
                receivers: vec![],
                counter: 0usize,
                is_closed: false,
            }),
        }
    }
}

impl<T> Channel<T> {
    pub fn add_receiver(capacity: Capacity, this: &Arc<Channel<T>>) -> Receiver<T> {
        let mut inner = this.inner.lock().unwrap();

        let index = inner.counter;
        inner.counter += 1;

        let shared = Arc::new(ReceiverShared::<T>::new(capacity, inner.is_closed));

        inner.receivers.push((index, shared.clone()));

        Receiver {
            shared,
            channel_index: index,
            capacity,
            channel: this.clone(),
        }
    }

    fn remove_receiver(&self, index: usize) {
        let mut inner = self.inner.lock().unwrap();
        inner.receivers.retain(|(i, _)| *i != index);
    }

    fn close(&self) {
        let mut inner = self.inner.lock().unwrap();

        inner.is_closed = true;

        for (_, recv) in inner.receivers.iter() {
            let mut recv_inner = recv.inner.lock().unwrap();
            recv_inner.closed = true;

            if let Some((tk, handle)) = &recv_inner.select_handle {
                handle.close(*tk);
            }

            recv.cv.notify_one();
        }
    }

    #[allow(dead_code)]
    fn num_receivers(&self) -> usize {
        let inner = self.inner.lock().unwrap();
        inner.receivers.len()
    }
}

#[derive(Debug)]
pub struct Receiver<T> {
    shared: Arc<ReceiverShared<T>>,
    channel_index: usize,
    capacity: Capacity,
    channel: Arc<Channel<T>>,
}

#[derive(Debug)]
struct ReceiverShared<T> {
    inner: Mutex<ReceiverInner<T>>,
    cv: Condvar,
}

impl<T> ReceiverShared<T> {
    fn write(&self, data: T) {
        let mut inner = self.inner.lock().unwrap();
        inner.buf.push(data);

        if let Some((tk, handle)) = &inner.select_handle {
            handle.update(*tk, inner.buf.len());
        }

        self.cv.notify_one();
    }
}

#[derive(Debug)]
struct ReceiverInner<T> {
    buf: Buffer<T>,
    closed: bool,
    select_handle: Option<(SelectToken, SelectGroup)>,
}

impl<T> ReceiverShared<T> {
    fn new(capacity: Capacity, closed: bool) -> Self {
        Self {
            inner: Mutex::new(ReceiverInner {
                buf: match capacity {
                    Capacity::Unbounded => Buffer::unbounded(),
                    Capacity::Bounded(capacity) => {
                        Buffer::bounded(NonZero::new(capacity.get()).unwrap())
                    }
                },
                closed,
                select_handle: None,
            }),
            cv: Condvar::default(),
        }
    }
}

impl<T> Clone for Receiver<T> {
    fn clone(&self) -> Self {
        self.clone_with_capacity(self.capacity)
    }
}

impl<T> Drop for Receiver<T> {
    fn drop(&mut self) {
        self.channel.remove_receiver(self.channel_index);
    }
}

impl<T> Receiver<T> {
    pub fn recv(&self) -> Result<T, ChannelError> {
        let inner = self.shared.inner.lock().unwrap();

        let mut inner = self
            .shared
            .cv
            .wait_while(inner, |inner| inner.buf.is_empty() && !inner.closed)
            .unwrap();

        if inner.closed && inner.buf.is_empty() {
            if let Some((tk, handle)) = &inner.select_handle {
                handle.ack_close(*tk);
            }

            Err(ChannelError::Closed)
        } else {
            if let Some((tk, handle)) = &inner.select_handle {
                debug_assert_ne!(inner.buf.len(), 0);

                handle.update(*tk, inner.buf.len() - 1);
            }
            Ok(inner.buf.dequeue().unwrap())
        }
    }

    pub fn try_recv(&self) -> Result<T, ChannelError> {
        let mut inner = self.shared.inner.lock().unwrap();

        if inner.closed && inner.buf.is_empty() {
            if let Some((tk, handle)) = &inner.select_handle {
                handle.ack_close(*tk);
            }
            Err(ChannelError::Closed)
        } else if inner.buf.is_empty() {
            Err(ChannelError::Empty)
        } else {
            if let Some((tk, handle)) = &inner.select_handle {
                handle.update(*tk, inner.buf.len() - 1);
            }
            Ok(inner.buf.dequeue().unwrap())
        }
    }

    pub fn clone_with_capacity(&self, capacity: Capacity) -> Self {
        Channel::<T>::add_receiver(capacity, &self.channel)
    }

    pub fn capacity(&self) -> Capacity {
        self.capacity
    }
}

impl<T> Selectable for Receiver<T> {
    fn register(&self, token: SelectToken, handle: SelectGroup) {
        let mut inner = self.shared.inner.lock().unwrap();

        debug_assert!(inner.select_handle.is_none());

        inner.select_handle = Some((token, handle));
    }

    fn unregister(&self) {
        let mut inner = self.shared.inner.lock().unwrap();

        debug_assert!(inner.select_handle.is_some());

        inner.select_handle = None;
    }
}

#[derive(Debug)]
pub struct Sender<T> {
    channel: Arc<Channel<T>>,
}

impl<T> Drop for Sender<T> {
    fn drop(&mut self) {
        self.channel.close();
    }
}

impl<T: Clone> Sender<T> {
    pub fn send(&self, val: T) {
        self.channel.write(val);
    }
}

impl<T> Sender<T> {
    pub fn get_channel(&self) -> Arc<Channel<T>> {
        self.channel.clone()
    }
}

pub fn channel<T>(capacity: Capacity) -> (Sender<T>, Receiver<T>) {
    let channel = Arc::new(Channel::<T>::default());

    let receiver = Channel::<T>::add_receiver(capacity, &channel);
    let sender = Sender { channel };

    (sender, receiver)
}

#[cfg(test)]
mod tests {
    use std::{sync::Weak, thread, time::Duration};

    use super::*;

    #[test]
    fn test_simple_channel() {
        let (s, r_recv) = channel::<f32>(Capacity::Bounded(NonZero::new(2).unwrap()));

        let r_try = r_recv.clone();

        assert_eq!(r_try.try_recv(), Err(ChannelError::Empty));

        s.send(1.1);
        assert_eq!(r_recv.recv(), Ok(1.1));
        assert_eq!(r_try.try_recv(), Ok(1.1));

        s.send(1.2);
        assert_eq!(r_recv.recv(), Ok(1.2));
        assert_eq!(r_try.try_recv(), Ok(1.2));

        assert_eq!(r_try.try_recv(), Err(ChannelError::Empty));
    }

    #[test]
    fn test_capacity() {
        let (s, r) = channel::<f32>(Capacity::Bounded(NonZero::new(2).unwrap()));
        s.send(1.1);
        s.send(1.2);

        assert_eq!(r.recv(), Ok(1.1));
        assert_eq!(r.recv(), Ok(1.2));

        assert_eq!(r.try_recv(), Err(ChannelError::Empty));

        s.send(1.1);
        s.send(1.2);
        s.send(1.3);

        assert_eq!(r.recv(), Ok(1.2));
        assert_eq!(r.recv(), Ok(1.3));
    }

    #[test]
    fn test_unbounded() {
        let (s, r) = channel::<f32>(Capacity::Unbounded);
        for i in 0..1000 {
            s.send(i as f32 * 0.1);
        }

        for i in 0..1000 {
            assert_eq!(r.recv(), Ok(i as f32 * 0.1));
        }

        assert_eq!(r.try_recv(), Err(ChannelError::Empty));
    }

    #[test]
    fn test_different_clone_capacity() {
        let (s, r) = channel::<f32>(Capacity::Bounded(NonZero::new(2).unwrap()));

        let r3 = r.clone_with_capacity(Capacity::Bounded(NonZero::new(3).unwrap()));

        s.send(1.1);
        s.send(1.2);
        s.send(1.3);

        assert_eq!(r.recv(), Ok(1.2));
        assert_eq!(r.recv(), Ok(1.3));
        assert_eq!(r.try_recv(), Err(ChannelError::Empty));

        assert_eq!(r3.recv(), Ok(1.1));
        assert_eq!(r3.try_recv(), Ok(1.2));
        assert_eq!(r3.recv(), Ok(1.3));
        assert_eq!(r.try_recv(), Err(ChannelError::Empty));
    }

    #[test]
    fn test_multiple_receiver() {
        let (s, r) = channel::<f32>(Capacity::Bounded(NonZero::new(2).unwrap()));

        s.send(1.1);
        assert_eq!(r.recv(), Ok(1.1));

        let r2 = r.clone();

        s.send(1.2);
        s.send(1.3);

        assert_eq!(r.recv(), Ok(1.2));
        assert_eq!(r.recv(), Ok(1.3));

        assert_eq!(r2.recv(), Ok(1.2));
        assert_eq!(r2.recv(), Ok(1.3));

        drop(s);
        assert_eq!(r.recv(), Err(ChannelError::Closed));
        assert_eq!(r2.recv(), Err(ChannelError::Closed));
    }

    #[test]
    fn test_drop_receivers() {
        let (s, r) = channel::<f32>(Capacity::Bounded(NonZero::new(2).unwrap()));

        assert_eq!(s.channel.num_receivers(), 1);

        let r2 = r.clone();

        assert_eq!(s.channel.num_receivers(), 2);

        let r3 = r.clone_with_capacity(Capacity::Bounded(NonZero::new(3).unwrap()));

        assert_eq!(s.channel.num_receivers(), 3);

        drop(r);
        drop(r2);
        drop(r3);

        assert_eq!(s.channel.num_receivers(), 0);

        // Send still works fine
        s.send(1.0);
    }

    #[test]
    fn test_drop_and_readd_receiver() {
        let (s, r) = channel::<f32>(Capacity::Bounded(NonZero::new(2).unwrap()));

        let w_channel: Weak<Channel<f32>> = Arc::downgrade(&s.channel);

        s.send(1.1);
        assert_eq!(r.try_recv(), Ok(1.1));

        drop(r);

        let channel = Weak::upgrade(&w_channel);
        assert!(channel.is_some());

        let r2 = Channel::<f32>::add_receiver(
            Capacity::Bounded(NonZero::new(3).unwrap()),
            &channel.unwrap(),
        );

        s.send(2.2);
        assert_eq!(r2.try_recv(), Ok(2.2));
    }

    #[test]
    fn test_thread_send() {
        let (s, r) = channel::<f32>(Capacity::Bounded(NonZero::new(2).unwrap()));

        let handle = thread::spawn(move || {
            thread::sleep(Duration::from_millis(50));
            s.send(1.1);
        });

        assert_eq!(r.recv(), Ok(1.1));

        handle.join().unwrap();
    }

    #[test]
    fn test_thread_drop() {
        let (s, r) = channel::<f32>(Capacity::Bounded(NonZero::new(2).unwrap()));

        let handle = thread::spawn(move || {
            thread::sleep(Duration::from_millis(50));
            drop(s);
        });

        assert_eq!(r.recv(), Err(ChannelError::Closed));

        handle.join().unwrap();
    }
}
