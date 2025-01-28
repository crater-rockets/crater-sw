use anyhow::Result;
use chrono::TimeDelta;
use crater::{
    crater::{logging::RerunLogger, sim::rocket::Rocket},
    nodes::{FtlOrderedExecutor, NodeConfig, NodeManager},
    parameters::ParameterService,
    telemetry::TelemetryService,
};
use log::{debug, info};
use std::{
    collections::HashMap, env, fs, thread::{self}, time::Instant
};

fn main() -> Result<()> {
    // Default log level to "info"
    if env::var("RUST_LOG").is_err() {
        env::set_var("RUST_LOG", "info")
    }
    pretty_env_logger::init();
    crater();

    let ts = TelemetryService::default();

    info!("Initializing Rerun logger...");
    let logger = RerunLogger::new(&ts)?;

    info!("Starting simulation thread");
    let simulation = thread::spawn(move || -> Result<()> {
        let params_file = "config/params.toml";

        info!("Reading parameters from '{params_file}'");

        let params_toml = fs::read_to_string(params_file)?;
        let params = ParameterService::from_toml(&params_toml)?;

        info!("Initalizing node manager");
        let mut nm = NodeManager::new(
            ts.clone(),
            params.clone(),
            HashMap::from([("rocket".to_string(), NodeConfig::default())]),
        );

        nm.add_node("rocket", |ctx| Ok(Box::new(Rocket::new("crater", ctx)?)))?;

        let dt_sec = params.get_f64("/sim/dt")?;
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

    info!("Boom!");

    Ok(())
}

fn crater() {
    println!("                             ____");
    println!("                     __,-~~/~    `---.");
    println!("                   _/_,---(      ,    )");
    println!("               __ /        <    /   )  \\___");
    println!("- ------===;;;'====--------CRATER----===;;;===----- -  -");
    println!("                  \\/  ~\"~\"~\"~\"~\"~\\~\"~)~\"/");
    println!("                  (_ (   \\  (     >    \\)");
    println!("                   \\_( _ <         >_>'");
    println!("                      ~ `-i' ::>|--\"");
    println!("                          I;|.|.|");
    println!("                         <|i::|i|`.");
    println!("                        (` ^'\"`-' \")");
}
