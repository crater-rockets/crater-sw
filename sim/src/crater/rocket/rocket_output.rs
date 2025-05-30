use crate::{
    core::time::Timestamp,
    crater::{aero::aerodynamics::AeroState, channels, engine::engine::RocketEngineMassProperties},
    nodes::NodeTelemetry,
    telemetry::TelemetrySender,
};

use anyhow::Result;
use crater_gnc::datatypes::gnc::NavigationOutput;

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
    snd_ideal_nav: TelemetrySender<NavigationOutput>,
}

impl RocketOutput {
    pub fn new(telemetry: &NodeTelemetry) -> Result<Self> {
        Ok(Self {
            snd_state: telemetry.publish(channels::rocket::STATE)?,
            snd_actions: telemetry.publish(channels::rocket::ACTIONS)?,
            snd_accels: telemetry.publish(channels::rocket::ACCEL)?,
            snd_aerostate: telemetry.publish(channels::rocket::AERO_STATE)?,
            snd_rocket_mass: telemetry.publish(channels::rocket::MASS_ROCKET)?,
            snd_engine_mass: telemetry.publish(channels::rocket::MASS_ENGINE)?,
            snd_ideal_nav: telemetry.publish(channels::gnc::IDEAL_NAV_OUTPUT)?,
        })
    }

    /// Updates outputs from the results of the latest step
    pub fn update(&self, t: Timestamp, rocket: &Rocket) {
        self.snd_state.send(t, rocket.state.clone());
        let t_s = t.monotonic.elapsed_seconds_f64();

        let ode_output = RocketOdeStep::calc(rocket, t_s, rocket.state.clone());

        let navout = NavigationOutput {
            pos_n_m: rocket.state.pos_n_m().cast::<f32>(),
            vel_n_m_s: rocket.state.vel_n_m_s().cast::<f32>(),
            quat_nb: rocket.state.quat_nb().cast::<f32>(),
            acc_unbias_b_m_s2: ode_output.accels.acc_b_m_s2.cast::<f32>(),
            angvel_unbias_b_rad_s: rocket.state.angvel_b_rad_s().cast::<f32>(),
        };

        self.snd_ideal_nav.send(t, navout);
        self.snd_actions.send(t, ode_output.actions);
        self.snd_accels.send(t, ode_output.accels);
        self.snd_aerostate.send(t, ode_output.aero_state);
        self.snd_rocket_mass.send(t, ode_output.mass_rocket);
        self.snd_engine_mass.send(t, ode_output.mass_engine);
    }
}
