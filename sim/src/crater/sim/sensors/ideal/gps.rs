use crate::{
    core::time::{Clock, Timestamp},
    crater::rocket::rocket_data::RocketState,
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryReceiver, TelemetrySender, Timestamped},
    utils::capacity::Capacity::Unbounded,
};
use anyhow::Result;
use chrono::TimeDelta;
use crater_gnc::datatypes::sensors::GpsSensorSample;

#[derive(Debug)]
pub struct IdealGPS {
    rx_state: TelemetryReceiver<RocketState>,

    tx_gps: TelemetrySender<GpsSensorSample>,
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

        let pos_n_m = state.pos_n_m();
        let vel_n_m_s = state.vel_n_m_s();

        let sample = GpsSensorSample {
            pos_n_m: pos_n_m.map(|v| v as f32),
            vel_n_m_s: vel_n_m_s.map(|v| v as f32),
        };

        self.tx_gps.send(Timestamp::now(clock), sample);

        Ok(StepResult::Continue)
    }
}
