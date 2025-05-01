use std::{
    fs,
    path::{Path, PathBuf},
    sync::{atomic::AtomicUsize, mpsc::Sender, Arc},
    thread::available_parallelism,
    time::{Duration, Instant},
};

use anyhow::Result;
use chrono::{format, TimeDelta};
use log::info;
use rand::{rngs::OsRng, TryRngCore};
use serde::Serialize;

use crate::{
    crater::logging::rerun::{RerunLogConfig, RerunLoggerBuilder},
    model::ModelBuilder,
    nodes::{FtlOrderedExecutor, NodeManager},
    parameters::{parameters, ParameterMap},
    telemetry::TelemetryService,
};

#[derive(Debug, Clone, Serialize)]
struct MonteCarloResult {
    index: usize,
    thread_id: usize,
    seed: u64,
    sim_duration_us: i64,
    log_duration_us: i64,
    log_file: PathBuf,
}

fn worker(
    model: impl ModelBuilder,
    params: ParameterMap,
    log_config: impl RerunLogConfig,
    thread_id: usize,
    run_index: Arc<AtomicUsize>,
    num_runs: usize,
    tx_result: Sender<MonteCarloResult>,
    out_dir: &Path,
) -> Result<()> {
    loop {
        let index = run_index.fetch_add(1, std::sync::atomic::Ordering::Relaxed);

        if index >= num_runs {
            return Ok(());
        }

        let seed = OsRng {}.try_next_u64().unwrap();

        let ts = TelemetryService::default();

        let mut log_builder = RerunLoggerBuilder::new(&ts);
        log_config.subscribe_telem(&mut log_builder)?;

        let mut nm = NodeManager::new(
            ts,
            params.clone(),
            crate::nodes::ParameterSampling::Random,
            seed,
        );

        model.build(&mut nm)?;


        let dt_sec = params.get_param("sim.dt")?.value_float()?;
        let dt = (dt_sec * 1000000.0) as i64;

        let start_time = Instant::now();
        FtlOrderedExecutor::run_blocking(nm, TimeDelta::microseconds(dt))?;
        let sim_duration = Instant::now() - start_time;


        let start_time = Instant::now();
        let mut rec = rerun::RecordingStreamBuilder::new("crater")
            .save(out_dir.join(format!("mc_{index:04}.rrd")))?;

        log_config.init_rec(&mut rec)?;
        let logger = log_builder.build(rec)?;

        logger.log_blocking()?;

        let log_duration = Instant::now() - start_time;

        let result = MonteCarloResult {
            index,
            thread_id,
            seed,
            sim_duration_us: sim_duration.as_micros() as i64,
            log_duration_us: log_duration.as_micros() as i64,
            log_file: PathBuf::new(),
        };

        tx_result.send(result)?;
    }
}

pub struct MonteCarloRunner<M, L> {
    num_workers: usize,
    num_runs: usize,
    params: ParameterMap,
    model_builder: M,
    log_config: L,
    out_dir: PathBuf,
}

impl<M, L> MonteCarloRunner<M, L>
where
    M: ModelBuilder + Clone + Send + 'static,
    L: RerunLogConfig + Clone + Send + 'static,
{
    pub fn new(
        model_builder: M,
        params: &Path,
        log_config: L,
        num_runs: usize,
        num_workers: Option<usize>,
        out_dir: PathBuf,
    ) -> Result<Self> {
        info!("Reading parameters from '{}'", params.display());

        let params_toml = fs::read_to_string(params)?;
        let params = parameters::parse_string(params_toml)?;

        let num_workers = num_workers.unwrap_or_else(|| available_parallelism().unwrap().get());

        info!("Montecarlo configuration: {num_workers} workers, {num_runs} runs");

        Ok(MonteCarloRunner {
            num_workers,
            num_runs,
            params,
            model_builder,
            log_config,
            out_dir,
        })
    }

    pub fn run_blocking(self) -> Result<()> {
        info!("Running Monte Carlo simulation!");

        let (tx_result, rx_result) = std::sync::mpsc::channel();
        let mut workers = vec![];

        let run_index = Arc::new(AtomicUsize::new(0));

        for i in 0..self.num_workers {
            let model = self.model_builder.clone();
            let params = self.params.clone();
            let log_config = self.log_config.clone();
            let tx_result = tx_result.clone();
            let run_index = run_index.clone();
            let out_dir = self.out_dir.clone();

            let worker = std::thread::spawn(move || {
                worker(
                    model,
                    params,
                    log_config,
                    i,
                    run_index,
                    self.num_runs,
                    tx_result,
                    &out_dir,
                )
            });

            workers.push(worker);
        }
        drop(tx_result);

        // Write the results to csv
        let out_file = self.out_dir.join("montecarlo.csv");
        let mut writer = csv::Writer::from_path(out_file)?;

        while let Ok(result) = rx_result.recv() {
            info!(
                "Run {} (thread {}) completed in {:.3} s (log: {:.3} s). Seed: {}",
                result.index,
                result.thread_id,
                result.sim_duration_us as f64 / 1_000_000.0,
                result.log_duration_us as f64 / 1_000_000.0,
                result.seed
            );

            writer.serialize(result)?;
        }

        for worker in workers {
            worker.join().unwrap()?;
        }

        Ok(())
    }
}
