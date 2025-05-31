use std::{
    any::{type_name, Any},
    collections::HashMap,
    sync::{Arc, Mutex},
};

use flume::{bounded, unbounded, Receiver, RecvError, Sender, TryRecvError};
use thiserror::Error;

use crate::{core::time::Timestamp, utils::capacity::Capacity};

#[derive(PartialEq, Eq, Error, Debug)]
pub enum TelemetryError {
    #[error("Requested channel type '{requested}', but channel is a '{expected}'")]
    WrongChannelDataType { requested: String, expected: String },

    #[error("Wrong channel type requested (MPMC / SPMC)")]
    WrongChannelType,

    #[error("Trying to read from an empty channel")]
    Empty,

    #[error("Trying to read from a closed channel")]
    Disconnected,

    #[error("Cannot create more than one producer for a channel")]
    AlreadyHasProducer,

    #[error("Provided channel name is not valid")]
    InvalidChannelName,
}

#[derive(Debug, Clone, Eq, PartialEq)]
pub struct Timestamped<T>(pub Timestamp, pub T);

#[derive(Debug)]
pub struct TelemetrySender<T> {
    transport: Arc<TelemetryChannelTransportInner<T>>,
}

impl<T: 'static + Clone> TelemetrySender<T> {
    pub fn send(&self, timestamp: Timestamp, value: T) {
        let senders = self.transport.to_rx_channels.lock().unwrap();

        for tx in senders.iter() {
            let _ = tx.send(Timestamped(timestamp, value.clone()));
        }
    }
}

#[derive(Debug)]
pub struct TelemetryReceiver<T> {
    receiver: Receiver<Timestamped<T>>,
}

impl<T> TelemetryReceiver<T> {
    pub fn recv(&self) -> Result<Timestamped<T>, TelemetryError> {
        self.receiver.recv().map_err(|e| match e {
            RecvError::Disconnected => TelemetryError::Disconnected,
        })
    }

    pub fn try_recv(&self) -> Result<Timestamped<T>, TelemetryError> {
        self.receiver.try_recv().map_err(|e| match e {
            TryRecvError::Disconnected => TelemetryError::Disconnected,
            TryRecvError::Empty => TelemetryError::Empty,
        })
    }

    pub fn inner(&self) -> &Receiver<Timestamped<T>> {
        &self.receiver
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ChannelType {
    SpMc,
    MpMc,
}

#[derive(Debug)]
struct TelemetryChannel {
    #[allow(dead_code)]
    name: String,

    typename: String,

    transport: Box<dyn Any + Send>, // Box<TelemetryChannelTransport<T>>

    ch_type: ChannelType,
    num_producers: usize,
}

#[derive(Debug)]
struct TelemetryChannelTransport<T> {
    inner: Arc<TelemetryChannelTransportInner<T>>,
}

#[derive(Debug)]
struct TelemetryChannelTransportInner<T> {
    to_rx_channels: Mutex<Vec<Sender<Timestamped<T>>>>,
}

impl<T> Default for TelemetryChannelTransportInner<T> {
    fn default() -> Self {
        TelemetryChannelTransportInner {
            to_rx_channels: Mutex::new(Vec::new()),
        }
    }
}

impl TelemetryChannel {
    fn new<T: 'static + Send>(name: &str, ch_type: ChannelType) -> Self {
        let transport = TelemetryChannelTransport::<T> {
            inner: Arc::new(TelemetryChannelTransportInner::default()),
        };

        Self {
            name: name.to_string(),
            typename: type_name::<T>().to_string(),
            transport: Box::new(transport),
            ch_type,
            num_producers: 0,
        }
    }

    fn add_producer<T: 'static>(&mut self) -> Result<TelemetrySender<T>, TelemetryError> {
        self.num_producers += 1;
        let transport = self.transport_mut::<T>()?;

        Ok(TelemetrySender {
            transport: transport.inner.clone(),
        })
    }

    fn add_subscriber<T: 'static>(
        &mut self,
        capacity: Capacity,
    ) -> Result<TelemetryReceiver<T>, TelemetryError> {
        let transport = self.transport_mut::<T>()?;

        let (tx, rx) = match capacity {
            Capacity::Bounded(cap) => bounded(cap.get()),
            Capacity::Unbounded => unbounded(),
        };

        transport.inner.to_rx_channels.lock().unwrap().push(tx);

        Ok(TelemetryReceiver { receiver: rx })
    }

    #[allow(dead_code)]
    fn downcast_ref<T: 'static>(&self) -> Result<&TelemetryChannelTransport<T>, TelemetryError> {
        self.transport
            .downcast_ref::<TelemetryChannelTransport<T>>()
            .ok_or(TelemetryError::WrongChannelDataType {
                requested: type_name::<T>().to_string(),
                expected: self.typename.clone(),
            })
    }

    fn transport_mut<T: 'static>(
        &mut self,
    ) -> Result<&mut TelemetryChannelTransport<T>, TelemetryError> {
        self.transport
            .downcast_mut::<TelemetryChannelTransport<T>>()
            .ok_or(TelemetryError::WrongChannelDataType {
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

    pub fn publish<T: 'static + Send>(
        &self,
        channel_name: &str,
    ) -> Result<TelemetrySender<T>, TelemetryError> {
        self.publish_impl(channel_name, ChannelType::SpMc)
    }

    pub fn publish_mp<T: 'static + Send>(
        &self,
        channel_name: &str,
    ) -> Result<TelemetrySender<T>, TelemetryError> {
        self.publish_impl(channel_name, ChannelType::MpMc)
    }

    fn publish_impl<T: 'static + Send>(
        &self,
        channel_name: &str,
        ch_type: ChannelType,
    ) -> Result<TelemetrySender<T>, TelemetryError> {
        // Remap the channel if needed
        let mut inner = self.inner.lock().unwrap();
        let channel_name = inner
            .remap
            .get(channel_name)
            .map(|v| v.clone())
            .or(Some(channel_name.to_string()))
            .unwrap();

        let channel = inner.get_channel::<T>(channel_name.as_str(), ch_type);

        match channel {
            Some(channel) => {
                if channel.ch_type == ChannelType::MpMc || channel.num_producers == 0 {
                    channel.add_producer()
                } else {
                    Err(TelemetryError::AlreadyHasProducer)
                }
            }
            None => Err(TelemetryError::WrongChannelType),
        }
    }

    pub fn subscribe<T: 'static + Send>(
        &self,
        channel_name: &str,
        capacity: Capacity,
    ) -> Result<TelemetryReceiver<T>, TelemetryError> {
        self.subscribe_impl(channel_name, capacity, ChannelType::SpMc)
    }

    pub fn subscribe_mp<T: 'static + Send>(
        &self,
        channel_name: &str,
        capacity: Capacity,
    ) -> Result<TelemetryReceiver<T>, TelemetryError> {
        self.subscribe_impl(channel_name, capacity, ChannelType::MpMc)
    }

    fn subscribe_impl<T: 'static + Send>(
        &self,
        channel_name: &str,
        capacity: Capacity,
        ch_type: ChannelType,
    ) -> Result<TelemetryReceiver<T>, TelemetryError> {
        let mut inner = self.inner.lock().unwrap();
        let channel = inner.get_channel::<T>(channel_name, ch_type);

        channel
            .ok_or(TelemetryError::WrongChannelType)?
            .add_subscriber(capacity)
    }
}

impl TelemetryServiceInner {
    fn get_channel<'a, T: 'static + Send>(
        &'a mut self,
        channel_name: &str,
        ch_type: ChannelType,
    ) -> Option<&'a mut TelemetryChannel> {
        if !self.channels.contains_key(channel_name) {
            self.channels.insert(
                channel_name.to_string(),
                TelemetryChannel::new::<T>(channel_name, ch_type),
            );
        }

        let ch = self.channels.get_mut(channel_name).unwrap();

        if ch.ch_type == ch_type {
            Some(ch)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{core::time::SystemClock};

    use super::*;

    #[test]
    fn test_empty_chan() -> Result<(), TelemetryError> {
        let telem_service = TelemetryService::default();

        let sub1 = telem_service.subscribe::<f64>("/test/channel/1", 1usize.into())?;

        assert_eq!(sub1.try_recv(), Err(TelemetryError::Empty));

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

        assert_eq!(sub1.try_recv(), Err(TelemetryError::Empty));
        assert_eq!(sub2.try_recv(), Err(TelemetryError::Empty));

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
        assert_eq!(s_ch1.try_recv(), Err(TelemetryError::Empty));
        assert_eq!(s_ch2.try_recv(), Ok(Timestamped(ts, 1.0)));
        assert_eq!(s_ch3.try_recv(), Err(TelemetryError::Empty));

        p_ch3.send(ts, 1.0);
        assert_eq!(s_ch1.try_recv(), Ok(Timestamped(ts, 1.0)));
        assert_eq!(s_ch2.try_recv(), Err(TelemetryError::Empty));
        assert_eq!(s_ch3.try_recv(), Err(TelemetryError::Empty));

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
            TelemetryError::WrongChannelDataType {
                requested: std::any::type_name::<f32>().to_string(),
                expected: std::any::type_name::<f64>().to_string()
            }
        );

        telem_service.publish::<f32>("/test/channel/2")?;
        let sub2 = telem_service.subscribe::<f64>("/test/channel/2", 1usize.into());

        assert!(sub2.is_err());
        assert_eq!(
            std::mem::discriminant::<TelemetryError>(&sub2.err().unwrap()),
            std::mem::discriminant::<TelemetryError>(&TelemetryError::WrongChannelDataType {
                requested: std::any::type_name::<f64>().to_string(),
                expected: std::any::type_name::<f32>().to_string()
            })
        );

        Ok(())
    }
}
