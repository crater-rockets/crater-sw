use std::{
    marker::PhantomData,
    sync::{Arc, Condvar, Mutex},
    thread::{self, ThreadId},
};

use rand::{rngs::ThreadRng, seq::IteratorRandom};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SelectToken {
    index: usize,
}

#[derive(Debug, Clone, Default)]
pub struct SelectHandle {
    inner: Arc<SelectHandleInner>,
}

#[derive(Debug, Default)]
struct SelectHandleInner {
    ready_list: Mutex<Vec<(usize, bool)>>,
    cv: Condvar,
}

impl SelectHandle {
    pub fn notify(&self, token: SelectToken) {
        let mut ready_list = self.inner.ready_list.lock().unwrap();

        ready_list.get_mut(token.index).unwrap().0 += 1;

        self.inner.cv.notify_one();
    }

    pub fn read(&self, token: SelectToken) {
        let mut ready_list = self.inner.ready_list.lock().unwrap();

        debug_assert_ne!(ready_list.get_mut(token.index).unwrap().0, 0);

        ready_list.get_mut(token.index).unwrap().0 -= 1;
    }

    pub fn close(&self, token: SelectToken) {
        let mut ready_list = self.inner.ready_list.lock().unwrap();

        ready_list.get_mut(token.index).unwrap().1 = true;

        self.inner.cv.notify_one();
    }
}

pub trait Selectable {
    fn register(&self, id: ThreadId, token: SelectToken, handle: SelectHandle);

    fn unregister(&self, id: ThreadId);
}

pub struct Select<'a> {
    handle: SelectHandle,
    subscribers: Vec<(&'a dyn Selectable, SelectToken)>,
    rng: ThreadRng,
    id: ThreadId,

    /// Make sure we are not "Send", as we are using our thread's ThreadId as a key to a map
    not_send: PhantomData<*const ()>,
}

impl<'a> Default for Select<'a> {
    fn default() -> Self {
        Self {
            handle: SelectHandle::default(),
            subscribers: vec![],
            rng: ThreadRng::default(),
            id: thread::current().id(),
            not_send: PhantomData::default(),
        }
    }
}

impl<'a> Drop for Select<'a> {
    fn drop(&mut self) {
        for (s, _) in self.subscribers.iter() {
            s.unregister(self.id);
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

    pub fn add(&mut self, selectable: &'a dyn Selectable) {
        let index = self.subscribers.len();
        let tk = SelectToken { index };

        self.subscribers.push((selectable, tk));

        {
            let mut ready_list = self.handle.inner.ready_list.lock().unwrap();
            ready_list.push((0, false));
        }

        selectable.register(self.id, tk, self.handle.clone());
    }

    pub fn ready(&mut self) -> usize {
        let handle = self.handle.inner.as_ref();

        let ready_list = handle
            .cv
            .wait_while(handle.ready_list.lock().unwrap(), |r| {
                r.iter().all(|(n, closed)| *n == 0usize && !*closed)
            })
            .unwrap();

        let ready_index = ready_list
            .iter()
            .enumerate()
            .filter(|(_, (n, closed))| *n > 0usize || *closed)
            .map(|(i, _)| i)
            .choose(&mut self.rng)
            .unwrap();

        ready_index
    }
}

#[cfg(test)]
mod tests {
    use std::{thread, time::Duration};

    use super::Select;
    use crate::utils::ringchannel::channel::{channel, ChannelError};
    use anyhow::Result;

    #[test]
    fn test_select() -> Result<()> {
        let (s1, r1) = channel::<i32>(1);
        let (s2, r2) = channel::<i32>(1);
        let (s3, r3) = channel::<i32>(1);

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
        let (s1, r1) = channel::<i32>(1);
        let (_, r2) = channel::<i32>(1);

        let mut select = Select::default();
        select.add(&r1);
        select.add(&r2);

        drop(s1);
        assert_eq!(select.ready(), 0);

        Ok(())
    }

    #[test]
    fn test_select_ready_blocking() -> Result<()> {
        let (s1, r1) = channel::<i32>(1);
        let (_, r2) = channel::<i32>(1);

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
        let (_, r1) = channel::<i32>(1);
        let (s2, r2) = channel::<i32>(1);

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
}
