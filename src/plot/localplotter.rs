use std::{
    any::Any,
    sync::mpsc::{channel, Receiver, Sender},
    thread::{self, JoinHandle},
};

use anyhow::Result;
use prost_reflect::{
    Cardinality, DynamicMessage, FieldDescriptor, Kind, MessageDescriptor, ReflectMessage, Value,
};
use rust_data_inspector::{PlotSampleSender, PlotSignalError, PlotSignalSample, PlotSignals};

use crate::{
    telemetry::{TelemetryError, TelemetryReceiver, TelemetryService},
    utils::{
        ringchannel::{Select, Selectable},
        time::nsec_to_sec_f64,
    },
};

use super::plotter::{self, Plotter};

trait SelectableDowncastable: Selectable {
    fn as_any(&self) -> &dyn Any;

    fn upcast(&self) -> &dyn Selectable;
}

impl<T: 'static> SelectableDowncastable for TelemetryReceiver<T> {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn upcast(&self) -> &dyn Selectable {
        self
    }
}

type DynamicRecvFn =
    Box<dyn Fn(&dyn SelectableDowncastable) -> Result<DynamicMessage, TelemetryError> + Send>;

pub struct LocalPlotter {
    plotter: Plotter,
    receivers: Vec<(
        String,
        Box<dyn SelectableDowncastable + Send>,
        DynamicRecvFn,
    )>,

    ts: TelemetryService,
}

impl LocalPlotter {
    pub fn new(ts: TelemetryService) -> Self {
        LocalPlotter {
            plotter: Plotter::new(),
            receivers: Vec::default(),
            ts,
        }
    }

    pub fn register<T: ReflectMessage + Default + 'static>(
        &mut self,
        signals: &mut PlotSignals,
        channel: &str,
    ) -> Result<()> {
        let desc = T::default().descriptor();

        let telem_receiver = self.ts.subcribe::<T>(channel, 1)?;

        self.plotter.register(signals, channel, desc)?;
        self.receivers.push((
            channel.to_string(),
            Box::new(telem_receiver),
            Box::new(Self::recv_dynamic::<T>),
        ));

        Ok(())
    }

    fn receive_telemetry(
        mut plotter: Plotter,
        receivers: Vec<(
            String,
            Box<dyn SelectableDowncastable + Send>,
            DynamicRecvFn,
        )>,
    ) {
        let mut select = Select::default();

        for (_, r, _) in receivers.iter() {
            select.add(r.upcast());
        }

        loop {
            let index = select.ready();
            let (channel, r, recv_fn) = receivers.get(index).unwrap();

            let msg = recv_fn(r.as_ref()).unwrap();
            plotter.plot(channel.as_str(), msg).unwrap();
        }
    }

    fn recv_dynamic<T: ReflectMessage + 'static>(
        receiver: &dyn SelectableDowncastable,
    ) -> Result<DynamicMessage, TelemetryError> {
        let receiver = receiver
            .as_any()
            .downcast_ref::<TelemetryReceiver<T>>()
            .unwrap();

        Ok(receiver.try_recv()?.transcode_to_dynamic())
    }

    pub fn run(self) -> JoinHandle<()> {
        thread::spawn(|| Self::receive_telemetry(self.plotter, self.receivers))
    }
}
