use std::collections::HashMap;
use std::{fs, thread};

use ::quadcopter::core::time::{Clock, Instant, Timestamp};
use ::quadcopter::nodes::{FtlOrderedExecutor, NodeConfig, NodeContext, NodeManager, StepResult};
use ::quadcopter::parameters::ParameterService;
use ::quadcopter::plot::localplotter::LocalPlotter;
use ::quadcopter::telemetry::{TelemetryDispatcher, TelemetryReceiver, TelemetryService};
use ::quadcopter::DESCRIPTOR_POOL;
use ::quadcopter::{nodes::Node, telemetry::TelemetrySender};
use anyhow::Result;
use chrono::TimeDelta;
use once_cell::sync::OnceCell;

pub mod quadcopter {
    pub mod examples {
        pub mod cart {
            include!(concat!(env!("OUT_DIR"), "/crater.examples.cart.rs"));
        }
    }
}

use quadcopter::examples::cart::*;
use rust_data_inspector::{DataInspector, PlotSignals};
struct Cart {
    m: f32,
    dt: f32,
    duration: i64,
    start_t: OnceCell<Instant>,
    state: CartState,

    rcv_force: TelemetryReceiver<Force>,
    snd_state: TelemetrySender<CartState>,
}

impl Cart {
    fn new(ctx: NodeContext) -> Result<Self> {
        let snd_state = ctx.telemetry().publish::<CartState>("/cart/state")?;
        let rcv_force = ctx
            .telemetry()
            .subcribe::<Force>("/cart/force", 1usize.into())?;

        Ok(Cart {
            m: ctx.parameters().get_f32("/example/cart/mass")?,
            dt: ctx.parameters().get_f32("/example/dt")?,
            duration: ctx.parameters().get_i64("/example/duration_ms")?,
            start_t: OnceCell::new(),
            state: CartState {
                timestamp: 0,
                pos: ctx.parameters().get_f32("/example/cart/initial_cond/pos")?,
                vel: ctx.parameters().get_f32("/example/cart/initial_cond/vel")?,
            },
            rcv_force,
            snd_state,
        })
    }
}

impl Node for Cart {
    fn step(&mut self, clock: &dyn Clock) -> Result<StepResult> {
        let ts = Timestamp::now(clock);

        let f = if self.start_t.get().is_some() {
            self.rcv_force.recv()?.1
        } else {
            Force {
                timestamp: 0,
                f: 0.0,
            }
        };

        let _ = self.start_t.set(ts.monotonic);

        let acc = self.m * f.f - self.state.vel;
        self.state.vel = self.state.vel + acc * self.dt;
        self.state.pos = self.state.pos + self.state.vel * self.dt;
        self.state.timestamp = ts.monotonic.elapsed().num_nanoseconds().unwrap();

        self.snd_state.send(ts, self.state);

        if (ts.monotonic - self.start_t.get().unwrap().elapsed()).elapsed()
            > TimeDelta::milliseconds(self.duration)
        {
            Ok(StepResult::Stop)
        } else {
            Ok(StepResult::Continue)
        }
    }
}

struct PositionControl {
    rcv_state: TelemetryReceiver<CartState>,
    snd_force: TelemetrySender<Force>,

    target_pos: f32,
    kp: f32,
}

impl PositionControl {
    fn new(ctx: NodeContext) -> Result<Self> {
        let rcv_state = ctx.telemetry().subcribe("/cart/state", 1usize.into())?;
        let snd_force = ctx.telemetry().publish("/cart/force")?;

        Ok(Self {
            rcv_state,
            snd_force,
            target_pos: ctx.parameters().get_f32("/example/poscontrol/target")?,
            kp: ctx.parameters().get_f32("/example/poscontrol/kp")?,
        })
    }
}
impl Node for PositionControl {
    fn step(&mut self, clock: &dyn Clock) -> anyhow::Result<StepResult> {
        let state = self.rcv_state.recv()?.1;

        let f = -(state.pos - self.target_pos) * self.kp;

        self.snd_force.send(
            Timestamp::now(clock),
            Force {
                timestamp: clock.monotonic().elapsed().num_nanoseconds().unwrap(),
                f,
            },
        );

        Ok(StepResult::Continue)
    }
}

fn main() -> Result<()> {
    let ts = TelemetryService::default();
    let params_toml = fs::read_to_string("params.toml")?;
    let params = ParameterService::from_toml(&params_toml)?;
    let mut nm = NodeManager::new(
        ts.clone(),
        params.clone(),
        HashMap::from([
            ("cart".to_string(), NodeConfig::default()),
            ("pos_controller".to_string(), NodeConfig::default()),
        ]),
    );

    nm.add_node("cart", |ctx| Ok(Box::new(Cart::new(ctx)?)))?;
    nm.add_node("pos_controller", |ctx| {
        Ok(Box::new(PositionControl::new(ctx)?))
    })?;

    let mut signals = PlotSignals::default();
    let mut local_plotter = LocalPlotter::new(ts.clone());

    local_plotter.plot_channel::<CartState>(&mut signals, "/cart/state")?;
    local_plotter.plot_channel::<Force>(&mut signals, "/cart/force")?;
    local_plotter.run();

    // let exec = ThreadedExecutor::run(nm);

    let dt = (params.get_f32("/example/dt")? * 1000.0) as i64;
    let exec =
        thread::spawn(move || FtlOrderedExecutor::run_blocking(nm, TimeDelta::milliseconds(dt)));
    DataInspector::run_native("plotter", signals).unwrap();

    exec.join().unwrap()?;

    Ok(())
}
