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

#[derive(Debug, Clone, Default)]
pub struct SelectGroup {
    inner: Arc<SelectGroupInner>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CloseState {
    Open,
    CloseSignaled,
    CloseAcknowldged,
}

#[derive(Debug, Default)]
struct SelectGroupInner {
    ready_list: Mutex<Vec<(usize, CloseState)>>,
    cv: Condvar,
}

impl SelectGroup {
    pub fn update(&self, token: SelectToken, nelem: usize) {
        let mut ready_list = self.inner.ready_list.lock().unwrap();

        ready_list.get_mut(token.index).unwrap().0 = nelem;

        self.inner.cv.notify_one();
    }

    pub fn close(&self, token: SelectToken) {
        let mut ready_list = self.inner.ready_list.lock().unwrap();

        ready_list.get_mut(token.index).unwrap().1 = CloseState::CloseSignaled;

        self.inner.cv.notify_one();
    }

    pub fn ack_close(&self, token: SelectToken) {
        let mut ready_list = self.inner.ready_list.lock().unwrap();

        ready_list.get_mut(token.index).unwrap().1 = CloseState::CloseAcknowldged;
    }
}

pub trait Selectable {
    fn register(&self, token: SelectToken, handle: SelectGroup);

    fn unregister(&self);
}

pub struct Select<'a> {
    handle: SelectGroup,
    subscribers: Vec<(&'a dyn Selectable, SelectToken)>,
    rng: ThreadRng,

    /// Make sure we are not "Send", as we are using our thread's ThreadId as a key to a map
    not_send: PhantomData<*const ()>,
}

impl<'a> Default for Select<'a> {
    fn default() -> Self {
        Self {
            handle: SelectGroup::default(),
            subscribers: vec![],
            rng: ThreadRng::default(),
            not_send: PhantomData::default(),
        }
    }
}

impl<'a> Drop for Select<'a> {
    fn drop(&mut self) {
        for (s, _) in self.subscribers.iter() {
            s.unregister();
        }
    }
}

impl<'a> Select<'a> {
    pub fn new(items: &[&'a dyn Selectable]) -> Self {
        let mut select = Select::default();
        for &s in items {
            select.add(s);
        }

        select
    }

    pub fn add(&mut self, selectable: &'a dyn Selectable) -> usize {
        let index = self.subscribers.len();
        let tk = SelectToken { index };

        self.subscribers.push((selectable, tk));

        {
            let mut ready_list = self.handle.inner.ready_list.lock().unwrap();
            ready_list.push((0, CloseState::Open));
        }

        selectable.register(tk, self.handle.clone());

        index
    }

    pub fn ready(&mut self) -> usize {
        let handle = self.handle.inner.as_ref();

        let ready_list = handle
            .cv
            .wait_while(handle.ready_list.lock().unwrap(), |r| {
                r.iter().all(|(n, close_state)| {
                    *n == 0usize && !(*close_state == CloseState::CloseSignaled)
                })
            })
            .unwrap();

        ready_list
            .iter()
            .enumerate()
            .filter(|(_, (n, close_state))| {
                *n > 0usize || (*close_state == CloseState::CloseSignaled)
            })
            .map(|(i, _)| i)
            .choose(&mut self.rng)
            .unwrap()
    }

    pub fn try_ready(&mut self) -> Result<usize, ChannelError> {
        let handle = self.handle.inner.as_ref();

        let ready_list = handle.ready_list.lock().unwrap();

        ready_list
            .iter()
            .enumerate()
            .filter(|(_, (n, close_state))| {
                *n > 0usize || (*close_state == CloseState::CloseSignaled)
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
    fn test_select_closed() -> Result<()> {
        let (s1, r1) = channel::<i32>(Capacity::Bounded(NonZero::new(1).unwrap()));
        let (_, r2) = channel::<i32>(Capacity::Bounded(NonZero::new(1).unwrap()));

        let mut select = Select::default();
        select.add(&r1);
        select.add(&r2);

        drop(s1);
        assert_eq!(select.ready(), 0);
        assert_eq!(select.try_ready(), Ok(0));

        assert_eq!(r1.recv(), Err(ChannelError::Closed));

        // Channel being closed is not reported anymore after the channel has been read
        assert_eq!(select.try_ready(), Err(ChannelError::Empty));

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
