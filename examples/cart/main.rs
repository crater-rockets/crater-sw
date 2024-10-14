use std::collections::HashMap;

use ::quadcopter::nodes::{Executor, NodeConfig, NodeContext, NodeManager, ThreadedExecutor};
use ::quadcopter::plot::localplotter::LocalPlotter;
use ::quadcopter::telemetry::{TelemetryDispatcher, TelemetryReceiver, TelemetryService};
use ::quadcopter::DESCRIPTOR_POOL;
use ::quadcopter::{nodes::Node, telemetry::TelemetrySender};
use anyhow::{anyhow, Result};

pub mod quadcopter {
    pub mod examples {
        pub mod cart {
            include!(concat!(env!("OUT_DIR"), "/quadcopter.examples.cart.rs"));
        }
    }
}

use quadcopter::examples::cart::*;
use rust_data_inspector::{DataInspector, PlotSignals};
struct Cart {
    m: f32,
    t: f32,
    dt: f32,

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
            m: 1.0,
            t: 0.0,
            dt: 0.01,
            state: CartState {
                timestamp: 0,
                pos: 0.0,
                vel: 1.0,
            },
            rcv_force,
            snd_state,
        })
    }
}

impl Node for Cart {
    fn step(&mut self) -> Result<()> {
        let f = if self.t > 0.0 {
            self.rcv_force.recv()?
        } else {
            Force {
                timestamp: 0,
                f: 0.0,
            }
        };

        self.t += self.dt;
        let acc = self.m * f.f - self.state.vel;
        self.state.vel = self.state.vel + acc * self.dt;
        self.state.pos = self.state.pos + self.state.vel * self.dt;
        self.state.timestamp = (self.t as f64 * 1000000000.0) as i64;

        self.snd_state.send(self.state);

        if self.t >= 10.0 {
            Err(anyhow!("Finish!"))
        } else {
            Ok(())
        }
    }
}

struct PositionControl {
    rcv_state: TelemetryReceiver<CartState>,
    snd_force: TelemetrySender<Force>,
}

impl PositionControl {
    fn new(ctx: NodeContext) -> Result<Self> {
        let rcv_state = ctx.telemetry().subcribe("/cart/state", 1usize.into())?;
        let snd_force = ctx.telemetry().publish("/cart/force")?;

        Ok(Self {
            rcv_state,
            snd_force,
        })
    }
}
impl Node for PositionControl {
    fn step(&mut self) -> anyhow::Result<()> {
        let state = self.rcv_state.recv()?;
        state.timestamp;

        let f = -(state.pos - 2.0) * 0.5;

        self.snd_force.send(Force {
            timestamp: state.timestamp,
            f,
        });

        Ok(())
    }
}

fn main() -> Result<()> {
    let ts = TelemetryService::default();
    let mut nm = NodeManager::new(
        ts.clone(),
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

    let exec = ThreadedExecutor::run(nm);
    DataInspector::run_native("plotter", signals).unwrap();

    exec.join()?;

    Ok(())
}
