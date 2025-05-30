use crate::{
    core::time::Timestamp,
    crater::{aero::aerodynamics::AeroState, engine::engine::RocketEngineMassProperties},
    nodes::NodeTelemetry,
    telemetry::TelemetrySender,
};

use anyhow::Result;

use super::{
    mass::RocketMassProperties,
    rocket::{Rocket, RocketOdeStep},
    rocket_data::{RocketAccelerations, RocketActions, RocketState},
};

// Outputs of the Rocket node
pub struct RocketOutput {
    snd_state: TelemetrySender<RocketState>,
    snd_actions: TelemetrySender<RocketActions>,
    snd_accels: TelemetrySender<RocketAccelerations>,
    snd_aerostate: TelemetrySender<AeroState>,
    snd_rocket_mass: TelemetrySender<RocketMassProperties>,
    snd_engine_mass: TelemetrySender<RocketEngineMassProperties>,
}

impl RocketOutput {
    pub fn new(telemetry: &NodeTelemetry) -> Result<Self> {
        Ok(Self {
            snd_state: telemetry.publish("/rocket/state")?,
            snd_actions: telemetry.publish("/rocket/actions")?,
            snd_accels: telemetry.publish("/rocket/accel")?,
            snd_aerostate: telemetry.publish("/rocket/aerostate")?,
            snd_rocket_mass: telemetry.publish("/rocket/mass/rocket")?,
            snd_engine_mass: telemetry.publish("/rocket/mass/engine")?,
        })
    }

    /// Updates outputs from the results of the latest step
    pub fn update(&self, t: Timestamp, rocket: &Rocket) {
        self.snd_state.send(t, rocket.state.clone());
        let t_s = t.monotonic.elapsed_seconds_f64();

        let ode_output = RocketOdeStep::calc(rocket, t_s, rocket.state.clone());

        self.snd_actions.send(t, ode_output.actions);
        self.snd_accels.send(t, ode_output.accels);
        self.snd_aerostate.send(t, ode_output.aero_state);
        self.snd_rocket_mass.send(t, ode_output.mass_rocket);
        self.snd_engine_mass.send(t, ode_output.mass_engine);
    }
}
