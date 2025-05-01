use std::cell::RefCell;

use crate::{
    core::time::Timestamp,
    telemetry::{TelemetryDispatcher, TelemetryReceiver, TelemetryService, Timestamped},
    utils::{capacity::Capacity, ringchannel::Select},
};

use anyhow::Result;
use rerun::RecordingStream;

pub trait RerunWrite {
    type Telem;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        ent_path: &str,
        ts: Timestamp,
        data: Self::Telem,
    ) -> Result<()>;
}

trait LogFunction {
    fn add_select<'a>(&'a self, select: &mut Select<'a>) -> usize;
    fn log(&self, rec: &mut RecordingStream) -> Result<bool>;
}

struct TelemetryLogFunction<T, L> {
    receiver: TelemetryReceiver<T>,
    data_logger: RefCell<L>,
    ent_path: String,
}

impl<T, L> TelemetryLogFunction<T, L> {
    fn new(receiver: TelemetryReceiver<T>, logger: L, ent_path: &str) -> Self {
        Self {
            receiver,
            data_logger: RefCell::new(logger),
            ent_path: ent_path.to_string(),
        }
    }
}

impl<T, L> LogFunction for TelemetryLogFunction<T, L>
where
    T: 'static + Send,
    L: RerunWrite<Telem = T>,
{
    fn add_select<'a>(&'a self, select: &mut Select<'a>) -> usize {
        select.add(&self.receiver)
    }

    fn log(&self, rec: &mut RecordingStream) -> Result<bool> {
        if let Ok(Timestamped(ts, state)) = self.receiver.recv() {
            self.data_logger
                .borrow_mut()
                .write(rec, &self.ent_path, ts, state)?;
            Ok(true)
        } else {
            Ok(false)
        }
    }
}

pub struct RerunLoggerBuilder {
    telem: TelemetryService,
    log_functions: Vec<Box<dyn LogFunction>>,
}

impl RerunLoggerBuilder {
    pub fn new(telem: &TelemetryService) -> Self {
        Self {
            telem: telem.clone(),
            log_functions: Vec::new(),
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

        self.log_functions.push(Box::new(log_fn));

        Ok(())
    }

    pub fn build(self, rec: RecordingStream) -> Result<RerunLogger> {
        Ok(RerunLogger {
            log_functions: self.log_functions,
            rec,
        })
    }
}

pub struct RerunLogger {
    log_functions: Vec<Box<dyn LogFunction>>,
    rec: RecordingStream,
}

impl RerunLogger {
    pub fn log_blocking(mut self) -> Result<()> {
        let mut select = Select::default();

        for log_fn in &self.log_functions {
            log_fn.add_select(&mut select);
        }

        while select.num_active_subs() > 0 {
            let i = select.ready();
            let log_fn = &self.log_functions[i];

            if !log_fn.log(&mut self.rec)? {
                select.remove(i);
            }
        }

        Ok(())
    }
}

pub trait RerunLogConfig {
    fn init_rec(&self, rec: &mut RecordingStream) -> Result<()>;

    fn subscribe_telem(&self, builder: &mut RerunLoggerBuilder) -> Result<()>;
}
