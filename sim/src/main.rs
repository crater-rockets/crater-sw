use anyhow::Result;
use chrono::TimeDelta;
use crater::{
    crater::{
        logging::RerunLogger,
        sim::{actuators::ideal::IdealServo, gnc::openloop::OpenloopControl, rocket::Rocket},
    },
    model::OpenLoopCrater,
    nodes::{FtlOrderedExecutor, NodeConfig, NodeManager},
    parameters,
    runner::{RngSeed, SingleThreadedRunner},
    telemetry::TelemetryService,
};
use log::info;
use std::{
    collections::HashMap,
    env, fs,
    path::Path,
    thread::{self},
    time::Instant,
};

fn build_model(nm: &mut NodeManager) -> Result<()> {
    nm.add_node("rocket", |ctx| Ok(Box::new(Rocket::new("crater", ctx)?)))?;
    nm.add_node("openloop_control", |ctx| {
        Ok(Box::new(OpenloopControl::new(ctx)?))
    })?;
    nm.add_node("ideal_servo", |ctx| Ok(Box::new(IdealServo::new(ctx)?)))?;

    Ok(())
}

fn main() -> Result<()> {
    // Default log level to "info"
    if env::var("RUST_LOG").is_err() {
        env::set_var("RUST_LOG", "info")
    }

    pretty_env_logger::init();
    crater();

    let runner = SingleThreadedRunner::new(
        OpenLoopCrater {},
        &Path::new("config/params.toml"),
        crater::nodes::ParameterSampling::Random,
        RngSeed::Rand,
    )?;

    runner.run_blocking()?;

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
