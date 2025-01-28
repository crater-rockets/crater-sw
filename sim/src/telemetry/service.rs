use std::{
    any::{type_name, Any},
    collections::HashMap,
    sync::{Arc, Mutex, Weak},
};

use thiserror::Error;

use crate::{
    core::time::Timestamp,
    utils::{
        capacity::Capacity,
        ringchannel::{
            channel, Channel, ChannelError, ReadyList, Receiver, SelectToken, Selectable, Sender,
        },
    },
};

#[derive(PartialEq, Eq, Error, Debug)]
pub enum TelemetryError {
    #[error("Requested channel type '{requested}', but channel is a '{expected}'")]
    WrongChannelType { requested: String, expected: String },

    #[error("Trying to read from an empty channel")]
    EmptyChannel,

    #[error("Trying to read from a closed channel")]
    ClosedChannel,

    #[error("Cannot create more than one producer for a channel")]
    AlreadyHasProducer,

    #[error("Provided channel name is not valid")]
    InvalidChannelName,
}

#[derive(Debug, Clone, Eq, PartialEq)]
pub struct Timestamped<T>(pub Timestamp, pub T);

#[derive(Debug)]
pub struct TelemetrySender<T> {
    sender: Sender<Timestamped<T>>,
}

impl<T: 'static + Clone> TelemetrySender<T> {
    pub fn send(&self, timestamp: Timestamp, value: T) {
        self.sender.send(Timestamped(timestamp, value));
    }
}

#[derive(Debug)]
pub struct TelemetryReceiver<T> {
    receiver: Receiver<Timestamped<T>>,
}

impl<T> TelemetryReceiver<T> {
    pub fn recv(&self) -> Result<Timestamped<T>, TelemetryError> {
        self.receiver.recv().map_err(|e| match e {
            ChannelError::Closed => TelemetryError::ClosedChannel,
            ChannelError::Empty => TelemetryError::EmptyChannel,
        })
    }

    pub fn try_recv(&self) -> Result<Timestamped<T>, TelemetryError> {
        self.receiver.try_recv().map_err(|e| match e {
            ChannelError::Closed => TelemetryError::ClosedChannel,
            ChannelError::Empty => TelemetryError::EmptyChannel,
        })
    }
}

#[derive(Debug)]
struct TelemetryChannel {
    #[allow(dead_code)]
    name: String,

    typename: String,

    channel: Box<dyn Any + Send>, // Box<TelemetryChannelTransport<T>>
}

struct TelemetryChannelTransport<T> {
    channel: Weak<Channel<Timestamped<T>>>,
    sender: Option<Sender<Timestamped<T>>>,
}

impl TelemetryChannel {
    fn new<T: 'static + Send>(name: &str) -> Self {
        let (sender, _) = channel::<Timestamped<T>>(Capacity::Unbounded);

        let transport = TelemetryChannelTransport::<T> {
            channel: Arc::downgrade(&sender.get_channel()),
            sender: Some(sender),
        };

        Self {
            name: name.to_string(),
            typename: type_name::<T>().to_string(),
            channel: Box::new(transport),
        }
    }

    fn take_producer<T: 'static>(&mut self) -> Result<TelemetrySender<T>, TelemetryError> {
        let channel = self.downcast_mut::<T>()?;

        Ok(TelemetrySender {
            sender: channel
                .sender
                .take()
                .ok_or(TelemetryError::AlreadyHasProducer)?,
        })
    }

    fn add_subscriber<T: 'static>(
        &mut self,
        capacity: Capacity,
    ) -> Result<TelemetryReceiver<T>, TelemetryError> {
        let channel = self.downcast_mut::<T>()?;

        let ch = Weak::upgrade(&channel.channel).ok_or(TelemetryError::ClosedChannel)?;

        Ok(TelemetryReceiver {
            receiver: Channel::<Timestamped<T>>::add_receiver(capacity, &ch),
        })
    }

    #[allow(dead_code)]
    fn downcast_ref<T: 'static>(&self) -> Result<&TelemetryChannelTransport<T>, TelemetryError> {
        self.channel
            .downcast_ref::<TelemetryChannelTransport<T>>()
            .ok_or(TelemetryError::WrongChannelType {
                requested: type_name::<T>().to_string(),
                expected: self.typename.clone(),
            })
    }

    fn downcast_mut<T: 'static>(
        &mut self,
    ) -> Result<&mut TelemetryChannelTransport<T>, TelemetryError> {
        self.channel
            .downcast_mut::<TelemetryChannelTransport<T>>()
            .ok_or(TelemetryError::WrongChannelType {
                requested: type_name::<T>().to_string(),
                expected: self.typename.clone(),
            })
    }
}

#[derive(Debug, Default, Clone)]
pub struct TelemetryService {
    inner: Arc<Mutex<TelemetryServiceInner>>,
}

#[derive(Debug, Default)]
pub struct TelemetryServiceInner {
    remap: HashMap<String, String>,
    channels: HashMap<String, TelemetryChannel>,
}

impl TelemetryService {
    pub fn new(remap: HashMap<String, String>) -> Self {
        TelemetryService {
            inner: Arc::new(Mutex::new(TelemetryServiceInner {
                remap,
                channels: HashMap::new(),
            })),
        }
    }
}

pub trait TelemetryDispatcher {
    fn publish<T: 'static + Send>(
        &self,
        channel_name: &str,
    ) -> Result<TelemetrySender<T>, TelemetryError>;

    fn subscribe<T: 'static + Send>(
        &self,
        channel_name: &str,
        capacity: Capacity,
    ) -> Result<TelemetryReceiver<T>, TelemetryError>;
}

impl TelemetryDispatcher for TelemetryService {
    fn publish<T: 'static + Send>(
        &self,
        channel_name: &str,
    ) -> Result<TelemetrySender<T>, TelemetryError> {
        // Remap the channel if needed
        let mut inner = self.inner.lock().unwrap();
        let channel_name = inner
            .remap
            .get(channel_name)
            .map(|v| v.clone())
            .or(Some(channel_name.to_string()))
            .unwrap();

        let channel = inner.get_channel::<T>(channel_name.as_str());

        channel.take_producer()
    }

    fn subscribe<T: 'static + Send>(
        &self,
        channel_name: &str,
        capacity: Capacity,
    ) -> Result<TelemetryReceiver<T>, TelemetryError> {
        let mut inner = self.inner.lock().unwrap();
        let channel = inner.get_channel::<T>(channel_name);

        channel.add_subscriber(capacity)
    }
}

impl TelemetryServiceInner {
    fn get_channel<'a, T: 'static + Send>(
        &'a mut self,
        channel_name: &str,
    ) -> &'a mut TelemetryChannel {
        if !self.channels.contains_key(channel_name) {
            self.channels.insert(
                channel_name.to_string(),
                TelemetryChannel::new::<T>(channel_name),
            );
        }

        self.channels.get_mut(channel_name).unwrap()
    }
}

impl<T> Selectable for TelemetryReceiver<T> {
    fn register(&self, token: SelectToken, ready_list: Arc<ReadyList>) {
        self.receiver.register(token, ready_list)
    }

    fn unregister(&self) {
        self.receiver.unregister()
    }
}

#[cfg(test)]
mod tests {
    use crate::{core::time::SystemClock, utils::ringchannel::Select};

    use super::*;

    #[test]
    fn test_empty_chan() -> Result<(), TelemetryError> {
        let telem_service = TelemetryService::default();

        let sub1 = telem_service.subscribe::<f64>("/test/channel/1", 1usize.into())?;

        assert_eq!(sub1.try_recv(), Err(TelemetryError::EmptyChannel));

        Ok(())
    }

    #[test]
    fn test_multiple_prod() -> Result<(), TelemetryError> {
        let telem_service = TelemetryService::default();

        telem_service.publish::<f64>("/test/channel/1")?;

        assert!(telem_service.publish::<f64>("/test/channel/1").is_err());

        Ok(())
    }

    #[test]
    fn test_pub_sub() -> Result<(), TelemetryError> {
        let telem_service = TelemetryService::default();

        let sub1 = telem_service.subscribe::<f64>("/test/channel/1", 1usize.into())?;
        let sub2 = telem_service.subscribe::<f64>("/test/channel/1", 1usize.into())?;

        let prod = telem_service.publish::<f64>("/test/channel/1")?;

        let ts = Timestamp::now(&SystemClock::default());

        prod.send(ts, 1.234);

        assert_eq!(sub1.try_recv(), Ok(Timestamped(ts, 1.234)));
        assert_eq!(sub2.try_recv(), Ok(Timestamped(ts, 1.234)));

        assert_eq!(sub1.try_recv(), Err(TelemetryError::EmptyChannel));
        assert_eq!(sub2.try_recv(), Err(TelemetryError::EmptyChannel));

        Ok(())
    }

    #[test]
    fn test_remap() -> Result<(), TelemetryError> {
        let remap = HashMap::from([
            ("/test/channel/1".to_string(), "/test/channel/2".to_string()),
            ("/test/channel/3".to_string(), "/test/channel/1".to_string()),
        ]);

        let telem_service = TelemetryService::new(remap);
        let s_ch1 = telem_service.subscribe::<f64>("/test/channel/1", 1usize.into())?;
        let s_ch2 = telem_service.subscribe::<f64>("/test/channel/2", 1usize.into())?;
        let s_ch3 = telem_service.subscribe::<f64>("/test/channel/3", 1usize.into())?;

        let p_ch1 = telem_service.publish::<f64>("/test/channel/1")?;
        let p_ch3 = telem_service.publish::<f64>("/test/channel/3")?;

        // Ch1 was remapped to 2, so publishing 2 again fails
        assert!(telem_service.publish::<f64>("/test/channel/2").is_err());

        let ts = Timestamp::now(&SystemClock::default());

        p_ch1.send(ts, 1.0);
        assert_eq!(s_ch1.try_recv(), Err(TelemetryError::EmptyChannel));
        assert_eq!(s_ch2.try_recv(), Ok(Timestamped(ts, 1.0)));
        assert_eq!(s_ch3.try_recv(), Err(TelemetryError::EmptyChannel));

        p_ch3.send(ts, 1.0);
        assert_eq!(s_ch1.try_recv(), Ok(Timestamped(ts, 1.0)));
        assert_eq!(s_ch2.try_recv(), Err(TelemetryError::EmptyChannel));
        assert_eq!(s_ch3.try_recv(), Err(TelemetryError::EmptyChannel));

        Ok(())
    }

    #[test]
    fn test_ring_buf() -> Result<(), TelemetryError> {
        let telem_service = TelemetryService::default();

        let sub = telem_service.subscribe::<f64>("/test/channel/1", 3usize.into())?;
        let prod = telem_service.publish::<f64>("/test/channel/1")?;

        let ts = Timestamp::now(&SystemClock::default());

        prod.send(ts, 1.0);
        prod.send(ts, 2.0);
        prod.send(ts, 3.0);

        assert_eq!(sub.try_recv(), Ok(Timestamped(ts, 1.0)));
        assert_eq!(sub.try_recv(), Ok(Timestamped(ts, 2.0)));
        assert_eq!(sub.try_recv(), Ok(Timestamped(ts, 3.0)));
        assert_eq!(sub.try_recv(), Err(TelemetryError::EmptyChannel));

        let ts = Timestamp::now(&SystemClock::default());

        prod.send(ts, 1.0);
        prod.send(ts, 2.0);
        prod.send(ts, 3.0);
        prod.send(ts, 4.0);

        assert_eq!(sub.try_recv(), Ok(Timestamped(ts, 2.0)));
        assert_eq!(sub.try_recv(), Ok(Timestamped(ts, 3.0)));
        assert_eq!(sub.try_recv(), Ok(Timestamped(ts, 4.0)));
        assert_eq!(sub.try_recv(), Err(TelemetryError::EmptyChannel));

        Ok(())
    }

    #[test]
    fn test_bad_channel_type() -> Result<(), TelemetryError> {
        let telem_service = TelemetryService::default();

        telem_service.subscribe::<f64>("/test/channel/1", 1usize.into())?;

        let pub1 = telem_service.publish::<f32>("/test/channel/1");

        assert!(pub1.is_err());
        assert_eq!(
            pub1.err().unwrap(),
            TelemetryError::WrongChannelType {
                requested: std::any::type_name::<f32>().to_string(),
                expected: std::any::type_name::<f64>().to_string()
            }
        );

        telem_service.publish::<f32>("/test/channel/2")?;
        let sub2 = telem_service.subscribe::<f64>("/test/channel/2", 1usize.into());

        assert!(sub2.is_err());
        assert_eq!(
            std::mem::discriminant::<TelemetryError>(&sub2.err().unwrap()),
            std::mem::discriminant::<TelemetryError>(&TelemetryError::WrongChannelType {
                requested: std::any::type_name::<f64>().to_string(),
                expected: std::any::type_name::<f32>().to_string()
            })
        );

        Ok(())
    }

    use anyhow::Result;

    #[test]
    fn test_select() -> Result<()> {
        let telem_service = TelemetryService::default();

        let prod1 = telem_service.publish::<f64>("/test/channel/1")?;
        let prod2 = telem_service.publish::<i32>("/test/channel/2")?;

        let sub1 = telem_service.subscribe::<f64>("/test/channel/1", 1usize.into())?;
        let sub2 = telem_service.subscribe::<i32>("/test/channel/2", 1usize.into())?;

        let mut select = Select::default();
        select.add(&sub1);
        select.add(&sub2);

        let ts = Timestamp::now(&SystemClock::default());

        prod1.send(ts, 1.1);
        assert_eq!(select.ready(), 0);
        sub1.recv()?;

        prod1.send(ts, 1.1);
        assert_eq!(select.ready(), 0);
        sub1.recv()?;

        prod2.send(ts, 1);
        assert_eq!(select.ready(), 1);
        sub2.recv()?;

        prod1.send(ts, 1.1);
        prod2.send(ts, 1);

        let r1 = select.ready();
        assert!([0usize, 1usize].contains(&r1));
        if r1 == 0 {
            sub1.recv()?;
        } else {
            sub2.recv()?;
        }

        let r2 = select.ready();
        assert!([0usize, 1usize].contains(&r2));
        assert_ne!(r1, r2);

        Ok(())
    }
}
