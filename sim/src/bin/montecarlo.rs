use anyhow::Result;
use crater::{
    crater::logging::rerun::CraterUiLogConfig, model::OpenLoopCrater,
    montecarlorunner::MonteCarloRunner,
};
use log::info;
use std::{
    env,
    path::{Path, PathBuf},
};

fn main() -> Result<()> {
    // Default log level to "info"
    if env::var("RUST_LOG").is_err() {
        unsafe { env::set_var("RUST_LOG", "info") }
    }

    pretty_env_logger::init();
    crater();

    let mut out_dir = PathBuf::from("out");
    // Create a directory with the current date and time
    out_dir.push(chrono::Local::now().format("%Y_%m_%d_%H-%M-%S").to_string());

    if !out_dir.exists() {
        std::fs::create_dir_all(&out_dir)?;
    }

    let runner = MonteCarloRunner::new(
        OpenLoopCrater {},
        &Path::new("config/params.toml"),
        CraterUiLogConfig,
        500,
        None,
        out_dir,
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
