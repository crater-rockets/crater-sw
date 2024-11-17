use std::{
    collections::HashMap,
    fs,
    sync::{atomic::AtomicBool, mpsc::channel, Arc, Mutex},
    thread,
};

use anyhow::Result;
use chrono::TimeDelta;
use quadcopter::{
    crater::sim::rocket::Rocket,
    crater_messages::sensors::{
        AeroAngles, AeroForces, AngularVelocity, EulerAngles, OrientationQuat, Position, Thrust,
        Velocity,
    },
    nodes::{FtlOrderedExecutor, NodeConfig, NodeManager},
    parameters::ParameterService,
    plot::localplotter::LocalPlotter,
    telemetry::TelemetryService,
};
use rust_data_inspector::{DataInspector, PlotSignals};

#[derive(Debug, Default, Clone)]
struct SimState {
    running: bool,
    restarting: bool
}

fn main() -> Result<()> {
    let mut signals = PlotSignals::default();
    let local_plotter = Arc::new(Mutex::new(LocalPlotter::new()));

    {
        let mut local_plotter = local_plotter.lock().unwrap();
        local_plotter.plot_channel::<Position>(&mut signals, "/rocket/position")?;
        local_plotter.plot_channel::<Velocity>(&mut signals, "/rocket/velocity_ned")?;
        local_plotter.plot_channel::<Velocity>(&mut signals, "/rocket/velocity_body")?;
        local_plotter.plot_channel::<AngularVelocity>(&mut signals, "/rocket/angular_vel")?;
        local_plotter.plot_channel::<Thrust>(&mut signals, "/rocket/thrust")?;
        local_plotter.plot_channel::<OrientationQuat>(&mut signals, "/rocket/orientation/quat")?;
        local_plotter.plot_channel::<EulerAngles>(&mut signals, "/rocket/orientation/euler")?;
        local_plotter.plot_channel::<AeroAngles>(&mut signals, "/rocket/aero/angles")?;
        local_plotter.plot_channel::<AeroForces>(&mut signals, "/rocket/aero/actions")?;
    }

    let (runsim_sender, runsim_receiver) = channel();
    let simstate = Arc::new(Mutex::new(SimState::default()));

    let crater = {
        let local_plotter: Arc<Mutex<LocalPlotter>> = local_plotter.clone();
        let simstate = simstate.clone();

        thread::spawn(move || -> Result<()> {
            while let Ok(_) = runsim_receiver.recv() {
                simstate.lock().unwrap().running = true;

                let ts = TelemetryService::default();
                let params_toml = fs::read_to_string("config/crater/params.toml")?;
                let params = ParameterService::from_toml(&params_toml)?;

                let mut nm = NodeManager::new(
                    ts.clone(),
                    params.clone(),
                    HashMap::from([("rocket".to_string(), NodeConfig::default())]),
                );

                nm.add_node("rocket", |ctx| Ok(Box::new(Rocket::new("crater", ctx)?)))?;

                let (stop_sender, stop_receiver) = channel();
                let plot_handle = local_plotter.lock().unwrap().run(&ts, stop_receiver)?;

                let dt = (params.get_f64("/sim/dt")? * 1000000.0) as i64;

                FtlOrderedExecutor::run_blocking(nm, TimeDelta::microseconds(dt))?;
                let _ = stop_sender.send(());
                plot_handle.join().unwrap()?;

                simstate.lock().unwrap().running = false;
            }

            Ok(())
        })
    };

    runsim_sender.send(())?;

    DataInspector::run_native(
        "plotter",
        signals,
        Some(move |ui: &mut egui::Ui, api: &mut rust_data_inspector::DataInspectorAPI| {
            let enabled = !simstate.lock().unwrap().running;
            if ui.add_enabled(enabled, egui::Button::new("↻ Restart")).clicked() {
                api.clear_timeseries();
                runsim_sender.send(()).unwrap();
            }
        }),
    )
    .unwrap();

    crater.join().unwrap()?;

    Ok(())
}
