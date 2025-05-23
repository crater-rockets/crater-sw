use crate::{
    core::time::{Clock, Timestamp},
    crater::sim::{rocket::rocket_data::RocketState, sensors::datatypes::GPSSample},
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryReceiver, TelemetrySender, Timestamped},
    utils::capacity::Capacity::Unbounded,
};
use anyhow::Result;
use chrono::TimeDelta;

#[derive(Debug)]
pub struct IdealGPS {
    rx_state: TelemetryReceiver<RocketState>,

    tx_gps: TelemetrySender<GPSSample>,
}

impl IdealGPS {
    pub fn new(ctx: NodeContext) -> Result<Self> {
        let rx_state = ctx.telemetry().subscribe("/rocket/state", Unbounded)?;

        let tx_gps = ctx.telemetry().publish("/sensors/gps")?;

        Ok(Self { rx_state, tx_gps })
    }
}

impl Node for IdealGPS {
    fn step(&mut self, _: usize, _: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
        let Timestamped(_, state) = self
            .rx_state
            .try_recv()
            .expect("GPS step executed, but no /rocket/state input available");

        let sample = GPSSample {
            pos_n: state.pos_n_m(),
            vel_n: state.vel_n_m_s(),
        };

        self.tx_gps.send(Timestamp::now(clock), sample);

        Ok(StepResult::Continue)
    }
}
