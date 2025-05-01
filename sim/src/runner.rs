use std::{
    fs,
    path::{Path, PathBuf},
    thread,
    time::{Duration, Instant},
};

pub use anyhow::Result;
use chrono::TimeDelta;
use log::info;
use rand::{rngs::OsRng, TryRngCore};

use crate::{
    crater::logging::rerun::{RerunLogConfig, RerunLoggerBuilder},
    model::ModelBuilder,
    nodes::{FtlOrderedExecutor, NodeManager, ParameterSampling},
    parameters::parameters,
    telemetry::TelemetryService,
};

pub enum LogOutput {
    Ui,
    File(PathBuf),
}

pub struct SingleThreadedRunner {
    nm: NodeManager,
    log_config: Box<dyn RerunLogConfig>,
    log_builder: RerunLoggerBuilder,
}

impl SingleThreadedRunner {
    pub fn new(
        model: impl ModelBuilder,
        params: &Path,
        log_config: Box<dyn RerunLogConfig>,
        param_sampling: ParameterSampling,
        seed: Option<u64>,
    ) -> Result<Self> {
        info!("Reading parameters from '{}'", params.display());

        let params_toml = fs::read_to_string(params)?;
        let params = parameters::parse_string(params_toml)?;

        let ts = TelemetryService::default();

        info!("Initalizing node manager");

        let seed = seed.unwrap_or(OsRng {}.try_next_u64().unwrap());
        let mut nm = NodeManager::new(ts.clone(), params.clone(), param_sampling, seed);

        model.build(&mut nm)?;

        let mut log_builder = RerunLoggerBuilder::new(&ts);
        log_config.subscribe_telem(&mut log_builder)?;

        Ok(Self {
            nm,
            log_builder,
            log_config,
        })
    }

    pub fn run_blocking(self) -> Result<()> {
        let params = self.nm.parameters();
        let nm = self.nm;
        let log_builder = self.log_builder;
        let log_config = self.log_config;

        let simulation = thread::spawn(move || -> Result<()> {
            let dt_sec = params.get_param("sim.dt")?.value_float()?;
            let dt = (dt_sec * 1000000.0) as i64;

            let dt_msec = dt_sec * 1000.0;

            info!("Simulation dt is {dt_msec:.2} ms");
            info!("Running simulation!");

            let start_time = Instant::now();
            FtlOrderedExecutor::run_blocking(nm, TimeDelta::microseconds(dt))?;

            let duration = (Instant::now() - start_time).as_secs_f64();

            info!("Simulation ended! Duration: {duration:.6} s");

            Ok(())
        });

        info!("Connecting to Rerun interface...");
        let mut rec = rerun::RecordingStreamBuilder::new("crater").connect_tcp()?;

        info!("Rerun connected!");
        log_config.init_rec(&mut rec)?;

        let logger = log_builder.build(rec)?;
        logger.log_blocking()?;

        info!("Rerun log completed");
        simulation.join().unwrap()?;

        Ok(())
    }
}
