use std::{
    fs,
    path::{Path, PathBuf},
    thread,
    time::{Duration, Instant},
};

pub use anyhow::Result;
use chrono::TimeDelta;
use log::info;

use crate::{
    crater::logging::RerunLogger,
    model::ModelBuilder,
    nodes::{FtlOrderedExecutor, NodeManager, ParameterSampling},
    parameters::parameters,
    telemetry::TelemetryService,
};

pub enum LogOutput {
    Ui,
    File(PathBuf),
}

pub enum RngSeed {
    Rand,
    Fixed(u64),
}

pub struct SingleThreadedRunner {
    nm: NodeManager,
    logger: RerunLogger,
}

impl SingleThreadedRunner {
    pub fn new(
        model: impl ModelBuilder,
        params: &Path,
        // log_out: LogOutput,
        param_sampling: ParameterSampling,
        seed: RngSeed,
    ) -> Result<Self> {
        info!("Reading parameters from '{}'", params.display());

        let params_toml = fs::read_to_string(params)?;
        let params = parameters::parse_string(params_toml)?;

        let ts = TelemetryService::default();

        info!("Initalizing node manager");
        let mut nm = match seed {
            RngSeed::Fixed(seed) => {
                NodeManager::new_from_seed(ts.clone(), params.clone(), param_sampling, seed)
            }
            RngSeed::Rand => NodeManager::new(ts.clone(), params.clone(), param_sampling),
        };

        model.build(&mut nm)?;

        let logger = RerunLogger::new(&ts)?;

        Ok(Self { nm, logger })
    }

    pub fn run_blocking(self) -> Result<()> {
        let params = self.nm.parameters();
        let nm = self.nm;
        let logger = self.logger;

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
        let mut log_conn = logger.connect()?;

        info!("Rerun connected!");
        log_conn.log_blocking()?;

        info!("Rerun log completed");
        simulation.join().unwrap()?;

        Ok(())
    }
}
