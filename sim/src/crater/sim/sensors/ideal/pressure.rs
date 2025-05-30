use crate::{
    core::time::{Clock, Timestamp},
    crater::{
        aero::atmosphere::{Atmosphere, AtmosphereIsa},
        rocket::rocket_data::RocketState,
    },
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryReceiver, TelemetrySender, Timestamped},
    utils::capacity::Capacity::Unbounded,
};
use anyhow::Result;
use chrono::TimeDelta;
use crater_gnc::datatypes::sensors::PressureSensorSample;

/// Implementation of an Ideal IMU, without noise or errors
#[derive(Debug)]
pub struct IdealStaticPressureSensor {
    rx_state: TelemetryReceiver<RocketState>,
    tx_pressure: TelemetrySender<PressureSensorSample>,
    atmosphere: AtmosphereIsa,
}

impl IdealStaticPressureSensor {
    pub fn new(ctx: NodeContext) -> Result<Self> {
        let rx_state = ctx.telemetry().subscribe("/rocket/state", Unbounded)?;

        let tx_pressure = ctx.telemetry().publish("/sensors/ideal_static_pressure")?;

        Ok(Self {
            rx_state,
            tx_pressure,
            atmosphere: AtmosphereIsa::default(),
        })
    }
}

impl Node for IdealStaticPressureSensor {
    fn step(&mut self, _: usize, _: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
        let Timestamped(_, state) = self
            .rx_state
            .try_recv()
            .expect("IMU step executed, but no /rocket/state input available");

        self.tx_pressure.send(
            Timestamp::now(clock),
            PressureSensorSample {
                pressure_pa: self.atmosphere.pressure_pa(-state.pos_n_m()[2]) as f32,
                temperature_degc: None,
            },
        );
        Ok(StepResult::Continue)
    }
}
