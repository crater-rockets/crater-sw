use anyhow::Result;
use chrono::TimeDelta;
use crater::{
    crater::{logging::RerunLogger, sim::rocket::Rocket},
    nodes::{FtlOrderedExecutor, NodeConfig, NodeManager},
    parameters::ParameterService,
    telemetry::TelemetryService,
};
use std::{
    collections::HashMap,
    fs,
    thread::{self},
};

fn main() -> Result<()> {
    let ts = TelemetryService::default();

    let logger = RerunLogger::new(&ts)?;

    let simulation = thread::spawn(move || -> Result<()> {
        let params_toml = fs::read_to_string("config/crater/params.toml")?;
        let params = ParameterService::from_toml(&params_toml)?;

        let mut nm = NodeManager::new(
            ts.clone(),
            params.clone(),
            HashMap::from([("rocket".to_string(), NodeConfig::default())]),
        );

        nm.add_node("rocket", |ctx| Ok(Box::new(Rocket::new("crater", ctx)?)))?;

        let dt = (params.get_f64("/sim/dt")? * 1000000.0) as i64;

        FtlOrderedExecutor::run_blocking(nm, TimeDelta::microseconds(dt))?;

        Ok(())
    });

    let mut log_conn = logger.connect()?;

    // while let Ok(_) = log_conn.log() {}

    simulation.join().unwrap()?;

    Ok(())
}
