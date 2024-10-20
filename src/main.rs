use std::{collections::HashMap, fs, thread};

use anyhow::Result;
use chrono::TimeDelta;
use quadcopter::{
    crater::sim::rocket::Rocket,
    crater_messages::sensors::{
        AeroForces, AeroState, AngularVelocity, EulerAngles, OrientationQuat, Position, Thrust,
        Velocity,
    },
    nodes::{FtlOrderedExecutor, NodeConfig, NodeManager},
    parameters::ParameterService,
    plot::localplotter::LocalPlotter,
    telemetry::TelemetryService,
};
use rust_data_inspector::{DataInspector, PlotSignals};

fn main() -> Result<()> {
    let ts = TelemetryService::default();
    let params_toml = fs::read_to_string("config/crater/params.toml")?;
    let params = ParameterService::from_toml(&params_toml)?;
    let mut nm = NodeManager::new(
        ts.clone(),
        params.clone(),
        HashMap::from([("rocket".to_string(), NodeConfig::default())]),
    );

    nm.add_node("rocket", |ctx| Ok(Box::new(Rocket::new("crater", ctx)?)))?;

    let mut signals = PlotSignals::default();
    let mut local_plotter = LocalPlotter::new(ts.clone());

    local_plotter.plot_channel::<Position>(&mut signals, "/rocket/position")?;
    local_plotter.plot_channel::<Velocity>(&mut signals, "/rocket/velocity_ned")?;
    local_plotter.plot_channel::<Velocity>(&mut signals, "/rocket/velocity_body")?;
    local_plotter.plot_channel::<AngularVelocity>(&mut signals, "/rocket/angular_vel")?;
    local_plotter.plot_channel::<Thrust>(&mut signals, "/rocket/thrust")?;
    local_plotter.plot_channel::<OrientationQuat>(&mut signals, "/rocket/orientation/quat")?;
    local_plotter.plot_channel::<EulerAngles>(&mut signals, "/rocket/orientation/euler")?;
    local_plotter.plot_channel::<AeroState>(&mut signals, "/rocket/aero/state")?;
    local_plotter.plot_channel::<AeroForces>(&mut signals, "/rocket/aero/actions")?;
    local_plotter.run();

    // let exec = ThreadedExecutor::run(nm);

    let dt = (params.get_f64("/sim/dt")? * 1000000.0) as i64;
    let exec =
        thread::spawn(move || FtlOrderedExecutor::run_blocking(nm, TimeDelta::microseconds(dt)));
    DataInspector::run_native("plotter", signals).unwrap();

    exec.join().unwrap()?;

    Ok(())
}
