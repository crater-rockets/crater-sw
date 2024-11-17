use std::{
    any::Any,
    sync::{
        mpsc::{Receiver, TryRecvError},
        Arc, Mutex,
    },
    thread::{self, JoinHandle},
};

use anyhow::Result;
use prost_reflect::{DynamicMessage, ReflectMessage};
use rust_data_inspector::PlotSignals;

use crate::{
    telemetry::{
        TelemetryDispatcher, TelemetryError, TelemetryReceiver, TelemetryService, Timestamped,
    },
    utils::{
        capacity::Capacity,
        ringchannel::{Select, Selectable},
    },
};

use super::{
    plotter::{self, Plotter},
    PlotterError,
};

trait SelectableAndAny: Selectable {
    fn as_any(&self) -> &dyn Any;

    fn upcast(&self) -> &dyn Selectable;
}

impl<T: 'static> SelectableAndAny for TelemetryReceiver<T> {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn upcast(&self) -> &dyn Selectable {
        self
    }
}

type DynamicRecvFn =
    fn(&dyn SelectableAndAny) -> Result<Timestamped<DynamicMessage>, TelemetryError>;

type TelemetrySubscriptionFn =
    dyn Fn(&TelemetryService) -> Result<Box<dyn SelectableAndAny + Send>, TelemetryError> + Send;

pub struct LocalPlotter {
    plotter: Arc<Mutex<Plotter>>,
    channels: Vec<(String, Box<TelemetrySubscriptionFn>, DynamicRecvFn)>,
}

impl LocalPlotter {
    pub fn new() -> Self {
        LocalPlotter {
            plotter: Arc::new(Mutex::new(Plotter::new())),
            channels: Vec::default(),
        }
    }

    pub fn plot_channel<T: ReflectMessage + Default + 'static>(
        &mut self,
        signals: &mut PlotSignals,
        channel: &str,
    ) -> Result<(), PlotterError> {
        let desc = T::default().descriptor();

        let channel_owned = channel.to_string();
        let sub_fn = move |ts: &TelemetryService|  -> Result<
            Box<dyn SelectableAndAny + Send>,
            TelemetryError,
        > {
            let receiver = ts.subscribe::<T>(channel_owned.as_str(), Capacity::Unbounded)?;
            Ok(Box::new(receiver))
        };

        self.plotter
            .lock()
            .unwrap()
            .register(signals, channel, desc)?;
        self.channels.push((
            channel.to_string(),
            Box::new(sub_fn),
            Self::recv_dynamic::<T>,
        ));

        Ok(())
    }

    fn receive_telemetry(
        plotter: Arc<Mutex<Plotter>>,
        receivers: Vec<(String, Box<dyn SelectableAndAny + Send>, DynamicRecvFn)>,
    ) -> Result<(), PlotterError> {
        let mut select = Select::default();

        for (_, r, _) in receivers.iter() {
            select.add(r.upcast());
        }

        // There's only one thread at a time, so we can keep this locked for the whole duration
        let mut plotter = plotter.lock().unwrap();
        let mut num_closed = 0usize;
        loop {
            let index = select.ready();
            let (channel, r, recv_fn) = receivers.get(index).unwrap();

            match recv_fn(r.as_ref()) {
                Ok(Timestamped(ts, msg)) => {
                    plotter.plot(channel.as_str(), ts.monotonic.elapsed_seconds_f64(), msg)?;
                }
                Err(TelemetryError::ClosedChannel) => {
                    num_closed += 1;
                    if num_closed == receivers.len() {
                        break;
                    }
                }
                Err(e) => {
                    panic!("Unexpected error when reading from telemetry: {:#?}", e);
                }
            }
        }

        Ok(())
    }

    pub fn run(&self, ts: &TelemetryService) -> Result<JoinHandle<Result<(), PlotterError>>> {
        let mut receivers: Vec<(String, Box<dyn SelectableAndAny + Send>, DynamicRecvFn)> = vec![];

        for (ch, sub_fn, recv_fn) in self.channels.iter() {
            receivers.push((ch.clone(), sub_fn(ts)?, recv_fn.clone()));
        }

        let plotter = self.plotter.clone();
        Ok(thread::spawn(move || -> Result<(), PlotterError> {
            Self::receive_telemetry(plotter, receivers)
        }))
    }

    fn recv_dynamic<T: ReflectMessage + 'static>(
        receiver: &dyn SelectableAndAny,
    ) -> Result<Timestamped<DynamicMessage>, TelemetryError> {
        let receiver = receiver
            .as_any()
            .downcast_ref::<TelemetryReceiver<T>>()
            .expect("Error downcast receiver to TelemetryReceiver<T>");

        let v = receiver.try_recv()?;
        Ok(Timestamped(v.0, v.1.transcode_to_dynamic()))
    }
}
