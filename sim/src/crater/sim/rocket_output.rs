use crate::{
    core::time::Timestamp,
    nodes::NodeTelemetry,
    telemetry::{TelemetryDispatcher, TelemetrySender},
};

use super::{
    aero::aerodynamics::{AeroState, Aerodynamics},
    engine::engine::{RocketEngine, RocketEngineMasses},
    gnc::ServoPosition,
    rocket_data::{AeroAngles, RocketActions, RocketMassProperties, RocketParams, RocketState},
};
use anyhow::Result;
use nalgebra::Vector3;

// Outputs of the Rocket node
pub struct RocketOutput {
    snd_state: TelemetrySender<RocketState>,
    snd_actions: TelemetrySender<RocketActions>,
    snd_aeroangles: TelemetrySender<AeroAngles>,
    snd_masses: TelemetrySender<RocketMassProperties>,
    snd_engine: TelemetrySender<RocketEngineMasses>
}

impl RocketOutput {
    pub fn new(telemetry: &NodeTelemetry) -> Result<Self> {
        Ok(Self {
            snd_state: telemetry.publish("/rocket/state")?,
            snd_actions: telemetry.publish("/rocket/actions")?,
            snd_aeroangles: telemetry.publish("/rocket/aero_angles")?,
            snd_masses: telemetry.publish("/rocket/masses")?,
            snd_engine: telemetry.publish("/rocket/engine")?
        })
    }

    /// Updates outputs from the results of the latest step
    pub fn update(
        &self,
        t: Timestamp,
        state: &RocketState,
        d_state: &RocketState, // State derivative over time
        servo_pos: &ServoPosition,
        engine: &dyn RocketEngine,
        _: &RocketParams,
        aerodynamics: &Aerodynamics,
        masses: &RocketMassProperties,
    ) {
        self.snd_state.send(t, state.clone());

        let aero = aerodynamics.calc(&AeroState::new(
            servo_pos.mix(),
            state.vel_b(&state.quat_nb()),
            Vector3::zeros(),
            state.angvel_b(),
            state.pos_n()[2],
        ));

        let actions = RocketActions {
            thrust_b: engine.thrust_b(t.monotonic.elapsed_seconds_f64()),
            aero_force_b: aero.forces,
            aero_torque_b: aero.moments,
            acc_n: d_state.vel_n(),
            acc_b: d_state.vel_b(&state.quat_nb()),
        };

        self.snd_actions.send(t, actions);

        self.snd_aeroangles.send(
            t,
            AeroAngles {
                alpha: aero.alpha,
                beta: aero.beta,
            },
        );

        self.snd_masses.send(t, masses.clone());

        self.snd_engine.send(t, engine.masses_prop(t.monotonic.elapsed_seconds_f64()).clone());
        
    }
}
