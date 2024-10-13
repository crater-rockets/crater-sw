use anyhow::Result;
use quadcopter::plot::localplotter::LocalPlotter;
use quadcopter::plot::PlotterError;
use quadcopter::quadcopter::sensors::{Magnetometer, Vec3};
use quadcopter::telemetry::{TelemetryDispatcher, TelemetryService};
use rand::Rng;
use rust_data_inspector::DataInspector;
use rust_data_inspector_signals::PlotSignals;
use std::f32::consts::PI;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

fn main() -> Result<()> {
    let mut signals = PlotSignals::default();

    let telemetry_service = TelemetryService::default();

    let mut local_plotter = LocalPlotter::new(telemetry_service.clone());

    local_plotter.register::<Magnetometer>(&mut signals, "/a/b/c")?;
    local_plotter.register::<Magnetometer>(&mut signals, "/one/two")?;

    let start = Instant::now();
    let mut rng = rand::thread_rng();
    let mut add_signal = |name: &str| {
        telemetry_producer(
            telemetry_service.clone(),
            name,
            rng.gen::<f32>() * 10.0 + 5.0,
            rng.gen(),
            rng.gen::<f32>() * PI * 2.0,
            rng.gen::<f32>() * 60f32 + 2f32,
            Some(start),
        );
    };

    let lph = local_plotter.run();

    add_signal("/a/b/c");
    add_signal("/one/two");

    DataInspector::run_native("plotter", signals).unwrap();

    match lph.join().unwrap() {
        Ok(_) => Ok(()),
        Err(PlotterError::Closed) => Ok(()),
        Err(e) => Err(e.into()),
    }
}

pub fn telemetry_producer(
    ts: TelemetryService,
    channel: &str,
    a: f32,
    f: f32,
    phi: f32,
    rate: f32,
    start_time: Option<Instant>,
) -> JoinHandle<()> {
    let sender = ts.publish::<Magnetometer>(channel).unwrap();

    thread::spawn(move || {
        let period_ms = u64::max((1000f32 / rate) as u64, 1);

        let start = start_time.unwrap_or(Instant::now());

        loop {
            let t = Instant::now() - start;
            let t = t.as_secs_f64();
            let m = Magnetometer {
                timestamp: (t * 1000000000.0) as i64,
                magn: Vec3 {
                    x: a * f32::sin(2f32 * PI * f * (t as f32) + phi),
                    y: a * f32::sin(2f32 * PI * f * (t as f32) + phi + PI / 2.0),
                    z: a * f32::sin(2f32 * PI * f * (t as f32) + phi + PI),
                },
            };

            sender.send(m);

            thread::sleep(Duration::from_millis(period_ms));
        }
    })
}
