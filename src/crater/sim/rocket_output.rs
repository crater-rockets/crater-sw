use crate::{
    core::time::Timestamp,
    nodes::NodeTelemetry,
    telemetry::{TelemetryDispatcher, TelemetrySender},
};

use super::{
    aero::aerodynamics::{AeroState, Aerodynamics},
    engine::engine::RocketEngine,
    rocket_data::{AeroAngles, RocketActions, RocketParams, RocketState},
};
use anyhow::Result;
use nalgebra::Vector3;

// Outputs of the Rocket node
pub struct RocketOutput {
    snd_state: TelemetrySender<RocketState>,
    snd_actions: TelemetrySender<RocketActions>,
    snd_aeroangles: TelemetrySender<AeroAngles>,
}

impl RocketOutput {
    pub fn new(telemetry: &NodeTelemetry) -> Result<Self> {
        Ok(Self {
            snd_state: telemetry.publish("/rocket/state")?,
            snd_actions: telemetry.publish("/rocket/actions")?,
            snd_aeroangles: telemetry.publish("/rocket/aero_angles")?,
        })
    }

    /// Updates outputs from the results of the latest step
    pub fn update(
        &self,
        t: Timestamp,
        state: &RocketState,
        engine: &dyn RocketEngine,
        _: &RocketParams,
        aerodynamics: &Aerodynamics,
    ) {
        self.snd_state.send(t, state.clone());

        let aero = aerodynamics.calc(&AeroState::new(
            state
                .quat_nb()
                .inverse_transform_vector(&state.vel_n().clone_owned()),
            Vector3::zeros(),
            state.angvel_b().clone_owned(),
            state.pos_n()[2],
        ));

        let actions = RocketActions {
            thrust_b: engine.thrust_b(t.monotonic.elapsed_seconds_f64()),
            aero_force_b: aero.forces,
            aero_torque_b: aero.moments,
        };

        self.snd_actions.send(t, actions);

        self.snd_aeroangles.send(
            t,
            AeroAngles {
                alpha: aero.alpha.to_degrees(),
                beta: aero.beta.to_degrees(),
            },
        );
    }
}
