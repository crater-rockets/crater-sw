use std::{
    any::{type_name, Any},
    collections::HashMap,
    sync::{Arc, Mutex, Weak},
};

use thiserror::Error;

use crate::utils::ringchannel::{
    channel, Channel, ChannelError, Receiver, Select, SelectToken, Selectable, Sender,
};

#[derive(PartialEq, Eq, Error, Debug)]
pub enum Error {
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

#[derive(Debug)]
pub struct TelemetryProducer<T> {
    sender: Sender<T>,
}

impl<T: 'static + Clone> TelemetryProducer<T> {
    pub fn send(&self, t: T) {
        self.sender.send(t);
    }
}

#[derive(Debug)]
pub struct TelemetrySubscriber<T> {
    receiver: Receiver<T>,
}

impl<T> TelemetrySubscriber<T> {
    pub fn recv(&self) -> Result<T, Error> {
        self.receiver.recv().map_err(|e| match e {
            ChannelError::Closed => Error::ClosedChannel,
            ChannelError::Empty => Error::EmptyChannel,
        })
    }

    pub fn try_recv(&self) -> Result<T, Error> {
        self.receiver.try_recv().map_err(|e| match e {
            ChannelError::Closed => Error::ClosedChannel,
            ChannelError::Empty => Error::EmptyChannel,
        })
    }
}

#[derive(Debug)]
struct TelemetryChannel {
    #[allow(dead_code)]
    name: String,

    typename: String,

    channel: Box<dyn Any>, // Box<TelemetryChannelTransport<T>>
}

struct TelemetryChannelTransport<T> {
    channel: Weak<Channel<T>>,
    sender: Option<Sender<T>>,
}

impl TelemetryChannel {
    fn new<T: 'static>(name: &str) -> Self {
        let (sender, _) = channel::<T>(1);

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

    fn take_producer<T: 'static>(&mut self) -> Result<TelemetryProducer<T>, Error> {
        let channel = self.downcast_mut::<T>()?;

        Ok(TelemetryProducer {
            sender: channel.sender.take().ok_or(Error::AlreadyHasProducer)?,
        })
    }

    fn add_subscriber<T: 'static>(
        &mut self,
        capacity: usize,
    ) -> Result<TelemetrySubscriber<T>, Error> {
        let channel = self.downcast_mut::<T>()?;

        let ch = Weak::upgrade(&channel.channel).ok_or(Error::ClosedChannel)?;

        Ok(TelemetrySubscriber {
            receiver: Channel::<T>::add_receiver(capacity, &ch),
        })
    }

    #[allow(dead_code)]
    fn downcast_ref<T: 'static>(&self) -> Result<&TelemetryChannelTransport<T>, Error> {
        self.channel
            .downcast_ref::<TelemetryChannelTransport<T>>()
            .ok_or(Error::WrongChannelType {
                requested: type_name::<T>().to_string(),
                expected: self.typename.clone(),
            })
    }

    fn downcast_mut<T: 'static>(&mut self) -> Result<&mut TelemetryChannelTransport<T>, Error> {
        self.channel
            .downcast_mut::<TelemetryChannelTransport<T>>()
            .ok_or(Error::WrongChannelType {
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

    pub fn publish<T: 'static>(
        &mut self,
        channel_name: &str,
    ) -> Result<TelemetryProducer<T>, Error> {
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

    pub fn subcribe<T: 'static>(
        &mut self,
        channel_name: &str,
        capacity: usize,
    ) -> Result<TelemetrySubscriber<T>, Error> {
        let mut inner = self.inner.lock().unwrap();
        let channel = inner.get_channel::<T>(channel_name);

        channel.add_subscriber(capacity)
    }
}

impl TelemetryServiceInner {
    fn get_channel<'a, T: 'static>(&'a mut self, channel_name: &str) -> &'a mut TelemetryChannel {
        if !self.channels.contains_key(channel_name) {
            self.channels.insert(
                channel_name.to_string(),
                TelemetryChannel::new::<T>(channel_name),
            );
        }

        self.channels.get_mut(channel_name).unwrap()
    }
}

impl<T> Selectable for TelemetrySubscriber<T> {
    fn register(
        &self,
        id: std::thread::ThreadId,
        token: SelectToken,
        handle: crate::utils::ringchannel::SelectHandle,
    ) {
        self.receiver.register(id, token, handle)
    }

    fn unregister(&self, id: std::thread::ThreadId) {
        self.receiver.unregister(id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_chan() -> Result<(), Error> {
        let mut telem_service = TelemetryService::default();

        let sub1 = telem_service.subcribe::<f64>("/test/channel/1", 1)?;

        assert_eq!(sub1.try_recv(), Err(Error::EmptyChannel));

        Ok(())
    }

    #[test]
    fn test_multiple_prod() -> Result<(), Error> {
        let mut telem_service = TelemetryService::default();

        telem_service.publish::<f64>("/test/channel/1")?;

        assert!(telem_service.publish::<f64>("/test/channel/1").is_err());

        Ok(())
    }

    #[test]
    fn test_pub_sub() -> Result<(), Error> {
        let mut telem_service = TelemetryService::default();

        let sub1 = telem_service.subcribe::<f64>("/test/channel/1", 1)?;
        let sub2 = telem_service.subcribe::<f64>("/test/channel/1", 1)?;

        let prod = telem_service.publish::<f64>("/test/channel/1")?;

        prod.send(1.234);

        assert_eq!(sub1.try_recv(), Ok(1.234));
        assert_eq!(sub2.try_recv(), Ok(1.234));

        assert_eq!(sub1.try_recv(), Err(Error::EmptyChannel));
        assert_eq!(sub2.try_recv(), Err(Error::EmptyChannel));

        Ok(())
    }

    #[test]
    fn test_remap() -> Result<(), Error> {
        let remap = HashMap::from([
            ("/test/channel/1".to_string(), "/test/channel/2".to_string()),
            ("/test/channel/3".to_string(), "/test/channel/1".to_string()),
        ]);

        let mut telem_service = TelemetryService::new(remap);
        let s_ch1 = telem_service.subcribe::<f64>("/test/channel/1", 1)?;
        let s_ch2 = telem_service.subcribe::<f64>("/test/channel/2", 1)?;
        let s_ch3 = telem_service.subcribe::<f64>("/test/channel/3", 1)?;

        let p_ch1 = telem_service.publish::<f64>("/test/channel/1")?;
        let p_ch3 = telem_service.publish::<f64>("/test/channel/3")?;

        // Ch1 was remapped to 2, so publishing 2 again fails
        assert!(telem_service.publish::<f64>("/test/channel/2").is_err());

        p_ch1.send(1.0);
        assert_eq!(s_ch1.try_recv(), Err(Error::EmptyChannel));
        assert_eq!(s_ch2.try_recv(), Ok(1.0));
        assert_eq!(s_ch3.try_recv(), Err(Error::EmptyChannel));

        p_ch3.send(1.0);
        assert_eq!(s_ch1.try_recv(), Ok(1.0));
        assert_eq!(s_ch2.try_recv(), Err(Error::EmptyChannel));
        assert_eq!(s_ch3.try_recv(), Err(Error::EmptyChannel));

        Ok(())
    }

    #[test]
    fn test_ring_buf() -> Result<(), Error> {
        let mut telem_service = TelemetryService::default();

        let sub = telem_service.subcribe::<f64>("/test/channel/1", 3)?;
        let prod = telem_service.publish::<f64>("/test/channel/1")?;

        prod.send(1.0);
        prod.send(2.0);
        prod.send(3.0);

        assert_eq!(sub.try_recv(), Ok(1.0));
        assert_eq!(sub.try_recv(), Ok(2.0));
        assert_eq!(sub.try_recv(), Ok(3.0));
        assert_eq!(sub.try_recv(), Err(Error::EmptyChannel));

        prod.send(1.0);
        prod.send(2.0);
        prod.send(3.0);
        prod.send(4.0);

        assert_eq!(sub.try_recv(), Ok(2.0));
        assert_eq!(sub.try_recv(), Ok(3.0));
        assert_eq!(sub.try_recv(), Ok(4.0));
        assert_eq!(sub.try_recv(), Err(Error::EmptyChannel));

        Ok(())
    }

    #[test]
    fn test_bad_channel_type() -> Result<(), Error> {
        let mut telem_service = TelemetryService::default();

        telem_service.subcribe::<f64>("/test/channel/1", 1)?;

        let pub1 = telem_service.publish::<f32>("/test/channel/1");

        assert!(pub1.is_err());
        assert_eq!(
            pub1.err().unwrap(),
            Error::WrongChannelType {
                requested: std::any::type_name::<f32>().to_string(),
                expected: std::any::type_name::<f64>().to_string()
            }
        );

        telem_service.publish::<f32>("/test/channel/2")?;
        let sub2 = telem_service.subcribe::<f64>("/test/channel/2", 1);

        assert!(sub2.is_err());
        assert_eq!(
            std::mem::discriminant::<Error>(&sub2.err().unwrap()),
            std::mem::discriminant::<Error>(&Error::WrongChannelType {
                requested: std::any::type_name::<f64>().to_string(),
                expected: std::any::type_name::<f32>().to_string()
            })
        );

        Ok(())
    }

    use anyhow::Result;

    #[test]
    fn test_select() -> Result<()> {
        let mut telem_service = TelemetryService::default();

        let prod1 = telem_service.publish::<f64>("/test/channel/1")?;
        let prod2 = telem_service.publish::<i32>("/test/channel/2")?;

        let sub1 = telem_service.subcribe::<f64>("/test/channel/1", 1)?;
        let sub2 = telem_service.subcribe::<i32>("/test/channel/2", 1)?;

        let mut select = Select::default();
        select.add(&sub1);
        select.add(&sub2);

        prod1.send(1.1);
        assert_eq!(select.ready(), 0);
        sub1.recv()?;

        prod1.send(1.1);
        assert_eq!(select.ready(), 0);
        sub1.recv()?;

        prod2.send(1);
        assert_eq!(select.ready(), 1);
        sub2.recv()?;

        prod1.send(1.1);
        prod2.send(1);

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
