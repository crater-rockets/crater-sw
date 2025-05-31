use anyhow::Result;
use crater::{
    crater::logging::rerun::CraterUiLogConfig, model::OpenLoopCrater, runner::SingleThreadedRunner,
};

use log::info;
use std::{env, path::Path};

fn main() -> Result<()> {
    // Default log level to "info"
    if env::var("RUST_LOG").is_err() {
        unsafe { env::set_var("RUST_LOG", "info") }
    }

    pretty_env_logger::init();
    crater();

    let runner = SingleThreadedRunner::new(
        OpenLoopCrater {},
        &Path::new("config/params.toml"),
        Box::new(CraterUiLogConfig),
        crater::nodes::ParameterSampling::Perfect,
        None,
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
