use crate::{core::time::Timestamp, nodes::NodeTelemetry, telemetry::TelemetrySender};

use super::{
    aero::aerodynamics::AeroState,
    engine::engine::RocketEngineMassProperties,
    gnc::ServoPosition,
    rocket::Rocket,
    rocket_data::{
        AeroAngles, RocketAccelerations, RocketActions, RocketMassProperties, RocketState,
    },
};
use anyhow::Result;
use nalgebra::Vector3;

// Outputs of the Rocket node
pub struct RocketOutput {
    snd_state: TelemetrySender<RocketState>,
    snd_actions: TelemetrySender<RocketActions>,
    snd_accels: TelemetrySender<RocketAccelerations>,
    snd_aeroangles: TelemetrySender<AeroAngles>,
    snd_masses: TelemetrySender<RocketMassProperties>,
    snd_engine: TelemetrySender<RocketEngineMassProperties>,
}

impl RocketOutput {
    pub fn new(telemetry: &NodeTelemetry) -> Result<Self> {
        Ok(Self {
            snd_state: telemetry.publish("/rocket/state")?,
            snd_actions: telemetry.publish("/rocket/actions")?,
            snd_accels: telemetry.publish("/rocket/accel")?,
            snd_aeroangles: telemetry.publish("/rocket/aero_angles")?,
            snd_masses: telemetry.publish("/rocket/masses")?,
            snd_engine: telemetry.publish("/rocket/engine")?,
        })
    }

    /// Updates outputs from the results of the latest step
    pub fn update(
        &self,
        t: Timestamp,
        state: &RocketState,
        d_state: &RocketState,
        servo_pos: &ServoPosition,
        rocket: &Rocket,
    ) {
        self.snd_state.send(t, state.clone());

        let aero = rocket.aerodynamics.calc(&AeroState::new(
            servo_pos.mix(),
            state.vel_b(&state.quat_nb()),
            Vector3::zeros(),
            state.angvel_b(),
            state.pos_n()[2],
        ));

        let engine_mass = rocket.engine.mass(
            rocket
                .fsm
                .t_from_ignition(t.monotonic.elapsed_seconds_f64()),
        );

        let mass_props = RocketMassProperties::calc_mass(&engine_mass, &rocket.params);

        let actions =
            rocket.calc_actions(t.monotonic.elapsed_seconds_f64(), state, &aero, &mass_props);

        self.snd_actions.send(t, actions);

        self.snd_accels.send(
            t,
            RocketAccelerations {
                acc_n: d_state.vel_n(),
                acc_b: d_state.vel_b(&state.quat_nb()),
                ang_acc_b: d_state.angvel_b(),
            },
        );

        self.snd_aeroangles.send(
            t,
            AeroAngles {
                alpha: aero.alpha,
                beta: aero.beta,
            },
        );

        self.snd_masses.send(t, mass_props.clone());
        self.snd_engine.send(t, engine_mass);
    }
}
