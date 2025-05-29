use std::cell::RefCell;

use crate::{
    core::time::Timestamp,
    telemetry::{TelemetryReceiver, TelemetryService, Timestamped},
    utils::capacity::Capacity,
};

use anyhow::Result;
use flume::Selector;
use rerun::RecordingStream;

pub trait RerunWrite {
    type Telem;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        timeline: &str,
        ent_path: &str,
        ts: Timestamp,
        data: Self::Telem,
    ) -> Result<()>;
}

trait SelectorReceiver {
    fn disconnected(&self) -> bool;

    fn recv<'a>(
        &'a mut self,
        selector: Selector<'a, ()>,
        rec: &'a RefCell<RecordingStream>,
    ) -> Selector<'a, ()>;
}

struct TelemetryLogFunction<T, L> {
    receiver: TelemetryReceiver<T>,
    data_logger: RefCell<L>,
    ent_path: String,
    disconnected: bool,
}

impl<T, L> TelemetryLogFunction<T, L> {
    fn new(receiver: TelemetryReceiver<T>, logger: L, ent_path: &str) -> Self {
        Self {
            receiver,
            data_logger: RefCell::new(logger),
            ent_path: ent_path.to_string(),
            disconnected: false,
        }
    }
}

impl<T, L> SelectorReceiver for TelemetryLogFunction<T, L>
where
    T: 'static + Send,
    L: RerunWrite<Telem = T>,
{
    fn disconnected(&self) -> bool {
        self.disconnected
    }

    fn recv<'a>(
        &'a mut self,
        selector: Selector<'a, ()>,
        rec: &'a RefCell<RecordingStream>,
    ) -> Selector<'a, ()> {
        selector.recv(self.receiver.inner(), |v| {
            if let Ok(Timestamped(ts, state)) = v {
                self.data_logger
                    .borrow_mut()
                    .write(&mut rec.borrow_mut(), "sim_time", &self.ent_path, ts, state)
                    .unwrap();
            } else {
                self.disconnected = true;
            }
        })
    }
}

// impl<T, L> LogFunction for TelemetryLogFunction<T, L>
// where
//     T: 'static + Send,
//     L: RerunWrite<Telem = T>,
// {
//     fn add_select<'a>(&'a self, select: &mut Select<'a>) -> usize {
//         select.add(&self.receiver)
//     }

//     fn log(&self, rec: &mut RecordingStream) -> Result<bool> {
//         if let Ok(Timestamped(ts, state)) = self.receiver.recv() {
//             self.data_logger
//                 .borrow_mut()
//                 .write(rec, &self.ent_path, ts, state)?;
//             Ok(true)
//         } else {
//             Ok(false)
//         }
//     }
// }

pub struct RerunLoggerBuilder {
    telem: TelemetryService,
    sel_receivers: Vec<Box<dyn SelectorReceiver>>,
}

impl RerunLoggerBuilder {
    pub fn new(telem: &TelemetryService) -> Self {
        Self {
            telem: telem.clone(),
            sel_receivers: Vec::new(),
        }
    }

    pub fn log_telemetry<T: 'static + Send>(
        &mut self,
        channel_name: &str,
        ent_path: &str,
        logger: impl RerunWrite<Telem = T> + 'static,
    ) -> Result<()> {
        let receiver = self
            .telem
            .subscribe::<T>(channel_name, Capacity::Unbounded)?;

        let log_fn = TelemetryLogFunction::new(receiver, logger, ent_path);

        self.sel_receivers.push(Box::new(log_fn));

        Ok(())
    }

    pub fn log_telemetry_mp<T: 'static + Send>(
        &mut self,
        channel_name: &str,
        ent_path: &str,
        logger: impl RerunWrite<Telem = T> + 'static,
    ) -> Result<()> {
        let receiver = self
            .telem
            .subscribe_mp::<T>(channel_name, Capacity::Unbounded)?;

        let log_fn = TelemetryLogFunction::new(receiver, logger, ent_path);

        self.sel_receivers.push(Box::new(log_fn));

        Ok(())
    }

    pub fn build(self, rec: RecordingStream) -> Result<RerunLogger> {
        Ok(RerunLogger {
            sel_receivers: self.sel_receivers,
            rec: RefCell::new(rec),
        })
    }
}

pub struct RerunLogger {
    sel_receivers: Vec<Box<dyn SelectorReceiver>>,
    rec: RefCell<RecordingStream>,
}

impl RerunLogger {
    pub fn log_blocking(mut self) -> Result<()> {
        loop {
            let mut selector: Selector<'_, ()> = Selector::new();
            let mut num_recv = 0usize;

            for sel_recv in self.sel_receivers.iter_mut() {
                if !sel_recv.disconnected() {
                    selector = sel_recv.recv(selector, &self.rec);
                    num_recv += 1;
                }
            }

            if num_recv > 0 {
                selector.wait();
            } else {
                break;
            }
        }

        Ok(())
    }
}

pub trait RerunLogConfig {
    fn init_rec(&self, rec: &mut RecordingStream) -> Result<()>;

    fn subscribe_telem(&self, builder: &mut RerunLoggerBuilder) -> Result<()>;
}
