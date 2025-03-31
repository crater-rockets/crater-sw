use crate::{
    core::time::{Clock, Timestamp},
    crater::sim::{rocket_data::RocketState, sensors::datatypes::MagnetometerSample},
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryDispatcher, TelemetryReceiver, TelemetrySender, Timestamped},
    utils::capacity::Capacity::Unbounded,
};
use anyhow::Result;
use chrono::TimeDelta;
use nalgebra::Vector3;

#[derive(Debug)]
pub struct IdealMagnetometer {
    rx_state: TelemetryReceiver<RocketState>,

    tx_magn: TelemetrySender<MagnetometerSample>,
}

impl IdealMagnetometer {
    pub fn new(ctx: NodeContext) -> Result<Self> {
        let rx_state = ctx.telemetry().subscribe("/rocket/state", Unbounded)?;

        let tx_magn = ctx.telemetry().publish("/sensors/magnetometer")?;

        Ok(Self { rx_state, tx_magn })
    }
}

impl Node for IdealMagnetometer {
    #[allow(unused)]
    fn step(&mut self, _: usize, _: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
        let Timestamped(_, state) = self
            .rx_state
            .try_recv()
            .expect("Magnetometer step executed, but no /rocket/state input available");

        let sample = MagnetometerSample {
            magfield_b: Vector3::<f64>::zeros(), // TODO: Implement magnetometer
        };

        self.tx_magn.send(Timestamp::now(clock), sample);

        Ok(StepResult::Continue)
    }
}
