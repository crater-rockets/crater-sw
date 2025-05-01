use std::{
    marker::PhantomData,
    sync::{Arc, Condvar, Mutex},
};

use rand::{rngs::ThreadRng, seq::IteratorRandom};

use super::ChannelError;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SelectToken {
    index: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SelectedReceiverState {
    Open,
    Closed,
    Removed,
}

#[derive(Debug, Default)]
pub struct ReadyList {
    receivers_state: Mutex<Vec<(usize, SelectedReceiverState)>>,
    cv: Condvar,
}

impl ReadyList {
    pub fn update(&self, token: SelectToken, nelem: usize) {
        let mut receivers_state = self.receivers_state.lock().unwrap();

        receivers_state.get_mut(token.index).unwrap().0 = nelem;

        self.cv.notify_one();
    }

    pub fn close(&self, token: SelectToken) {
        let mut receivers_state = self.receivers_state.lock().unwrap();

        receivers_state.get_mut(token.index).unwrap().1 = SelectedReceiverState::Closed;

        self.cv.notify_one();
    }
}

pub trait Selectable {
    fn register(&self, token: SelectToken, ready_list: Arc<ReadyList>);

    fn unregister(&self);

    fn state(&self) -> SelectedReceiverState;

    fn num_elem(&self) -> usize;
}

pub struct Select<'a> {
    ready_list: Arc<ReadyList>,
    subscribers: Vec<Option<(&'a dyn Selectable, SelectToken)>>,
    rng: ThreadRng,

    num_active_subs: usize,

    /// Make sure we are not "Send", as we are using our thread's ThreadId as a key to a map
    not_send: PhantomData<*const ()>,
}

impl<'a> Default for Select<'a> {
    fn default() -> Self {
        Self {
            ready_list: Arc::default(),
            subscribers: vec![],
            num_active_subs: 0usize,
            rng: ThreadRng::default(),
            not_send: PhantomData::default(),
        }
    }
}

impl<'a> Drop for Select<'a> {
    fn drop(&mut self) {
        for s in self.subscribers.iter() {
            if let Some((s, _)) = s {
                s.unregister();
            }
        }
    }
}

impl<'a> Select<'a> {
    /// Adds a selectable receiver to the Select object. The index of the selectable is returned:
    /// a call to ready() will return the same index if this receiver is ready
    pub fn add(&mut self, selectable: &'a dyn Selectable) -> usize {
        let index = self.subscribers.len();
        let tk = SelectToken { index };

        self.subscribers.push(Some((selectable, tk)));

        {
            let mut ready_list = self.ready_list.receivers_state.lock().unwrap();
            ready_list.push((0, selectable.state()));
        }

        selectable.register(tk, self.ready_list.clone());

        self.num_active_subs += 1;
        index
    }

    /// Removes an existing receiver
    /// Index of existing receivers will remain unchanged, and the index of the removed receiver will not be reused.
    ///
    /// # Panics
    /// Panics if the index does not correspond to any receiver
    pub fn remove(&mut self, index: usize) {
        if let Some(Some((s, _))) = self.subscribers.get(index) {
            let mut ready_list = self.ready_list.receivers_state.lock().unwrap();
            let state = ready_list
                .get_mut(index)
                .expect("ready_list and subscribers list mismatch!");
            state.0 = 0;
            state.1 = SelectedReceiverState::Removed;

            s.unregister();
            *self.subscribers.get_mut(index).unwrap() = None;

            self.num_active_subs -= 1;
        } else {
            panic!("No receiver with index {index} is being selected!");
        }
    }

    pub fn num_active_subs(&self) -> usize {
        self.num_active_subs
    }

    /// Blocks until one or more of the receivers is ready, returning its index.
    /// A receiver is "ready" if a call to recv() on that receiver does not black:
    /// that can be either because the channel is empty, or because the channel has been closed.
    /// If multiple receivers are ready, a random one among them is returned.
    ///
    /// # Panics
    /// If no receiver have been added to the Select object
    pub fn ready(&mut self) -> usize {
        if self.subscribers.is_empty() {
            panic!("ready() called on an empty Select object");
        }

        let ready_list = self.ready_list.as_ref();

        let ready_list = ready_list
            .cv
            .wait_while(ready_list.receivers_state.lock().unwrap(), |r| {
                r.iter().all(|(n, close_state)| {
                    *n == 0usize && !(*close_state == SelectedReceiverState::Closed)
                })
            })
            .unwrap();

        ready_list
            .iter()
            .enumerate()
            .filter(|(_, (n, close_state))| {
                *n > 0usize || (*close_state == SelectedReceiverState::Closed)
            })
            .map(|(i, _)| i)
            .choose(&mut self.rng)
            .unwrap()
    }

    /// Attempts to find a ready receiver.
    /// If one receiver is ready, its index is returned. If more receivers are ready, a random one among them is returned.
    /// If none are ready, an error is returned.
    pub fn try_ready(&mut self) -> Result<usize, ChannelError> {
        if self.subscribers.is_empty() {
            return Err(ChannelError::Empty);
        }

        let handle = self.ready_list.as_ref();

        let ready_list = handle.receivers_state.lock().unwrap();

        ready_list
            .iter()
            .enumerate()
            .filter(|(_, (n, close_state))| {
                *n > 0usize || (*close_state == SelectedReceiverState::Closed)
            })
            .map(|(i, _)| i)
            .choose(&mut self.rng)
            .ok_or(ChannelError::Empty)
    }
}

#[cfg(test)]
mod tests {
    use std::{num::NonZero, thread, time::Duration};

    use super::Select;
    use crate::utils::{
        capacity::Capacity,
        ringchannel::channel::{channel, ChannelError},
    };
    use anyhow::Result;

    #[test]
    fn test_select() -> Result<()> {
        let (s1, r1) = channel::<i32>(Capacity::Bounded(NonZero::new(1).unwrap()));
        let (s2, r2) = channel::<i32>(Capacity::Bounded(NonZero::new(1).unwrap()));
        let (s3, r3) = channel::<i32>(Capacity::Bounded(NonZero::new(1).unwrap()));

        let receivers = [r1, r2, r3];

        let mut select = Select::default();

        for r in receivers.iter() {
            select.add(r);
        }

        s1.send(1);

        assert_eq!(select.ready(), 0);
        assert_eq!(select.ready(), 0);

        receivers[0].recv()?;

        s2.send(1);
        assert_eq!(select.ready(), 1);
        assert_eq!(select.ready(), 1);
        receivers[1].recv()?;

        s2.send(1);
        s3.send(1);

        let ready = select.ready();
        assert!([1, 2].contains(&ready));
        receivers[ready].recv()?;

        let ready = select.ready();
        assert!([1, 2].contains(&ready));
        receivers[ready].recv()?;

        Ok(())
    }

    #[test]
    #[should_panic]
    fn test_select_no_receivers() {
        let mut select = Select::default();

        select.ready();
    }

    #[test]
    fn test_select_remove_while_ready() -> Result<()> {
        let (s1, r1) = channel::<i32>(Capacity::Bounded(NonZero::new(1).unwrap()));
        let (s2, r2) = channel::<i32>(Capacity::Bounded(NonZero::new(1).unwrap()));

        let mut select = Select::default();

        assert_eq!(select.add(&r1), 0);
        assert_eq!(select.add(&r2), 1);

        s1.send(1);
        s1.send(2);

        assert_eq!(select.try_ready(), Ok(0));

        select.remove(0);
        assert_eq!(select.try_ready(), Err(ChannelError::Empty));

        Ok(())
    }

    #[test]
    fn test_try_select_no_receivers() {
        let mut select = Select::default();

        assert_eq!(select.try_ready(), Err(ChannelError::Empty));
    }

    #[test]
    fn test_select_closed() -> Result<()> {
        let (s1, r1) = channel::<i32>(Capacity::Bounded(NonZero::new(1).unwrap()));
        let (s2, r2) = channel::<i32>(Capacity::Bounded(NonZero::new(1).unwrap()));

        let mut select = Select::default();
        select.add(&r1);
        select.add(&r2);

        // No channels ready
        assert_eq!(select.try_ready(), Err(ChannelError::Empty));

        // Test reception on the second channel
        s2.send(123);
        assert_eq!(select.ready(), 1);
        let _ = r2.recv();

        // No channels ready again
        assert_eq!(select.try_ready(), Err(ChannelError::Empty));

        // Close the first channel
        drop(s1);

        // First channel should become ready, as it is closed
        assert_eq!(select.ready(), 0);
        assert_eq!(select.try_ready(), Ok(0));

        assert_eq!(r1.recv(), Err(ChannelError::Closed));

        // A channel being closed means that it's always "ready"
        assert_eq!(select.ready(), 0);
        assert_eq!(select.try_ready(), Ok(0));

        Ok(())
    }

    #[test]
    fn test_select_ready_blocking() -> Result<()> {
        let (s1, r1) = channel::<i32>(Capacity::Bounded(NonZero::new(1).unwrap()));
        let (_, r2) = channel::<i32>(Capacity::Bounded(NonZero::new(1).unwrap()));

        let handle = thread::spawn(move || -> Result<usize, ChannelError> {
            let mut select = Select::default();
            select.add(&r1);
            select.add(&r2);

            Ok(select.ready())
        });

        thread::sleep(Duration::from_millis(50));

        s1.send(1);
        assert_eq!(handle.join().unwrap(), Ok(0));
        Ok(())
    }

    #[test]
    fn test_select_drop_blocking() -> Result<()> {
        let (_, r1) = channel::<i32>(Capacity::Bounded(NonZero::new(1).unwrap()));
        let (s2, r2) = channel::<i32>(Capacity::Bounded(NonZero::new(1).unwrap()));

        let handle = thread::spawn(move || -> Result<usize, ChannelError> {
            let mut select = Select::default();
            select.add(&r1);
            select.add(&r2);

            Ok(select.ready())
        });

        thread::sleep(Duration::from_millis(50));
        drop(s2);

        assert_eq!(handle.join().unwrap(), Ok(1));
        Ok(())
    }

    #[test]
    fn test_select_buf_overflow_try() -> Result<()> {
        let (s1, r1) = channel::<i32>(Capacity::Bounded(NonZero::new(3).unwrap()));

        let mut select = Select::default();
        select.add(&r1);

        s1.send(1);
        s1.send(2);
        s1.send(3);
        s1.send(4);

        assert_eq!(select.try_ready(), Ok(0));
        assert_eq!(r1.try_recv(), Ok(2));

        assert_eq!(select.try_ready(), Ok(0));
        assert_eq!(r1.try_recv(), Ok(3));

        assert_eq!(select.try_ready(), Ok(0));
        assert_eq!(r1.try_recv(), Ok(4));

        assert_eq!(select.try_ready(), Err(ChannelError::Empty));

        Ok(())
    }

    #[test]
    fn test_select_buf_overflow() -> Result<()> {
        let (s1, r1) = channel::<i32>(Capacity::Bounded(NonZero::new(3).unwrap()));

        let mut select = Select::default();
        select.add(&r1);

        s1.send(1);
        s1.send(2);
        s1.send(3);
        s1.send(4);

        assert_eq!(select.ready(), 0);
        assert_eq!(r1.recv(), Ok(2));

        assert_eq!(select.ready(), 0);
        assert_eq!(r1.recv(), Ok(3));

        assert_eq!(select.ready(), 0);
        assert_eq!(r1.recv(), Ok(4));

        assert_eq!(select.try_ready(), Err(ChannelError::Empty));

        Ok(())
    }
}
