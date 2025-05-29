use crater_gnc::{common::Timestamped, hal::channel::Full};
use embassy_sync::pubsub::{DynPublisher, DynSubscriber, WaitResult};

pub struct EmbassySender<'a, T: Clone>(DynPublisher<'a, Timestamped<T>>);

impl<'a, T: Clone> crater_gnc::hal::channel::Sender<T> for EmbassySender<'a, T> {
    fn try_send(&mut self, ts: crater_gnc::Instant, item: T) -> Result<(), Full<T>> {
        if let Err(v) = self.0.try_publish(Timestamped::new(ts, item)) {
            Err(Full(v))
        } else {
            Ok(())
        }
    }

    fn send_immediate(&mut self, ts: crater_gnc::Instant, item: T) {
        self.0.publish_immediate(Timestamped::new(ts, item));
    }
}

pub struct EmbassyReceiver<'a, T: Clone> {
    rx: DynSubscriber<'a, Timestamped<T>>,
    num_lagged: usize,
}

impl<'a, T: Clone> crater_gnc::hal::channel::Receiver<T> for EmbassyReceiver<'a, T> {
    fn try_recv(&mut self) -> Option<crater_gnc::common::Ts<T>> {
        loop {
            match self.rx.try_next_message() {
                Some(WaitResult::Lagged(n)) => {
                    self.num_lagged += n as usize;
                }
                Some(WaitResult::Message(msg)) => {
                    return Some(msg);
                }
                None => return None,
            }
        }
    }

    fn num_lagged(&self) -> usize {
        self.num_lagged
    }

    fn len(&self) -> usize {
        self.rx.available() as usize
    }

    fn capacity(&self) -> usize {
        self.rx.capacity()
    }

    fn is_empty(&self) -> bool {
        self.rx.is_empty()
    }

    fn is_full(&self) -> bool {
        self.rx.is_full()
    }
}

impl<'a, T: Clone> From<DynSubscriber<'a, Timestamped<T>>> for EmbassyReceiver<'a, T> {
    fn from(value: DynSubscriber<'a, Timestamped<T>>) -> Self {
        Self {
            num_lagged: 0,
            rx: value,
        }
    }
}

impl<'a, T: Clone> From<DynPublisher<'a, Timestamped<T>>> for EmbassySender<'a, T> {
    fn from(value: DynPublisher<'a, Timestamped<T>>) -> Self {
        Self(value)
    }
}
