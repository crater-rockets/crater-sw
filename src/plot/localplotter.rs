use std::{
    any::Any,
    thread::{self, JoinHandle},
};

use anyhow::Result;
use prost_reflect::{DynamicMessage, ReflectMessage};
use rust_data_inspector::PlotSignals;

use crate::{
    telemetry::{TelemetryDispatcher, TelemetryError, TelemetryReceiver, TelemetryService},
    utils::{capacity::Capacity, ringchannel::{Select, Selectable}},
};

use super::{plotter::Plotter, PlotterError};

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

    pub fn plot_channel<T: ReflectMessage + Default + 'static>(
        &mut self,
        signals: &mut PlotSignals,
        channel: &str,
    ) -> Result<(), PlotterError> {
        let desc = T::default().descriptor();

        let telem_receiver = self.ts.subcribe::<T>(channel, Capacity::Unbounded)?;

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
    ) -> Result<(), PlotterError> {
        let mut select = Select::default();

        for (_, r, _) in receivers.iter() {
            select.add(r.upcast());
        }

        loop {
            let index = select.ready();
            let (channel, r, recv_fn) = receivers.get(index).unwrap();

            match recv_fn(r.as_ref()) {
                Ok(msg) => {
                    plotter.plot(channel.as_str(), msg)?;
                }
                Err(TelemetryError::ClosedChannel) => {}
                Err(e) => {
                    panic!("Unexpected error when reading from telemetry: {:#?}", e);
                }
            }
        }
    }

    fn recv_dynamic<T: ReflectMessage + 'static>(
        receiver: &dyn SelectableDowncastable,
    ) -> Result<DynamicMessage, TelemetryError> {
        let receiver = receiver
            .as_any()
            .downcast_ref::<TelemetryReceiver<T>>()
            .expect("Error downcast receiver to TelemetryReceiver<T>");

        Ok(receiver.try_recv()?.transcode_to_dynamic())
    }

    pub fn run(self) -> JoinHandle<Result<(), PlotterError>> {
        thread::spawn(|| -> Result<(), PlotterError> {
            Self::receive_telemetry(self.plotter, self.receivers)
        })
    }
}
