use super::{
    engine::{engine::RocketEngine, SimpleRocketEngine},
    gnc::ServoPosition,
    rocket_data::{RocketParams, RocketState},
    rocket_output::RocketOutput,
};
use crate::{
    core::time::{Clock, Timestamp, TD},
    crater::sim::aero::{
        aerodynamics::{AeroCoefficients, AeroState, Aerodynamics},
        atmosphere::AtmosphereIsa,
    },
    math::ode::{OdeProblem, OdeSolver, RungeKutta4},
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryDispatcher, TelemetryReceiver, Timestamped},
    utils::capacity::Capacity::Unbounded,
};
use anyhow::{anyhow, Result};
use chrono::TimeDelta;
use core::f64;
use nalgebra::{Quaternion, SVector, Vector3, Vector4};

pub struct Rocket {
    params: RocketParams,
    state: RocketState,
    step_state: StepState,

    engine: Box<dyn RocketEngine + Send>,
    aerodynamics: Aerodynamics,

    rx_servo_pos: TelemetryReceiver<ServoPosition>,
    output: RocketOutput,
}

/// Variables allowed to change between steps, but not within a step (more precisely, during integration of a single step)
#[derive(Debug, Clone, Default)]
struct StepState {
    servo_pos: ServoPosition,
}

impl Rocket {
    pub fn new(name: &str, ctx: NodeContext) -> Result<Self> {
        // Base path for the parameters of this Rocket
        let param_path = format!("/sim/rocket/{name}");

        // Select which engine to use based on the config file (currently only one option)
        let engine = match ctx
            .parameters()
            .get_string(format!("{param_path}/engine/engine_type").as_str())?
            .as_str()
        {
            "simple" => Ok(Box::new(SimpleRocketEngine::from_impulse(
                ctx.parameters()
                    .get_f64(format!("{param_path}/engine/simple/total_impulse").as_str())?,
                ctx.parameters()
                    .get_f64(format!("{param_path}/engine/simple/thrust_duration").as_str())?,
            ))),
            unknown => Err(anyhow!(
                "Unknown engine type selected for rocket '{name}': {unknown}"
            )),
        }?;

        // Read parameters
        let params = RocketParams::from_service(&param_path, &ctx.parameters())?;

        // Initialize state with initial conditions from parameters
        let state = RocketState::from_params(&params);

        let atmosphere = Box::new(AtmosphereIsa::default());

        let aero_coefficients = AeroCoefficients::from_params(&param_path, &ctx.parameters())?;
        let aerodynamics = Aerodynamics::new(
            params.diameter,
            params.surface,
            atmosphere,
            aero_coefficients,
        );

        let rx_servo_pos = ctx
            .telemetry()
            .subscribe("/actuators/servo_position", Unbounded)?;
        let output = RocketOutput::new(ctx.telemetry())?;

        Ok(Rocket {
            engine,
            params,
            aerodynamics,
            state,
            rx_servo_pos,
            output,
            step_state: StepState::default(),
        })
    }
}

impl OdeProblem<f64, 13> for Rocket {
    fn odefun(&self, t: f64, y: SVector<f64, 13>) -> SVector<f64, 13> {
        let state = RocketState(y);

        // state derivative
        let mut dstate = RocketState::default();

        let q_nb = state.quat_nb();
        let vel_b = q_nb.inverse_transform_vector(&state.vel_n().clone_owned());
        let w_b = state.angvel_b();

        let aero = self.aerodynamics.calc(&AeroState::new(
            self.step_state.servo_pos.mix(),
            vel_b,
            Vector3::zeros(),
            w_b.clone_owned(),
            -state.pos_n()[2],
        ));

        let f_n = q_nb.transform_vector(&(self.engine.thrust_b(t) + &aero.forces));
        let m_b = aero.moments;

        let acc_n = f_n / self.params.mass + self.params.g_n;

        let qw =
            Quaternion::from_vector(Vector4::new(w_b[0] / 2.0, w_b[1] / 2.0, w_b[2] / 2.0, 0.0));
        let qdot = q_nb.into_inner() * qw;

        let w_dot = self.params.inv_inertia * (m_b + (self.params.inertia * w_b).cross(&w_b));

        dstate.set_pos_n(&state.vel_n());
        dstate.set_vel_n(&acc_n);
        dstate.set_quat_nb_vec(qdot.as_vector());
        dstate.set_angvel_b(&w_dot);

        dstate.0
    }
}

impl Node for Rocket {
    fn step(&mut self, i: usize, dt: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
        let t = Timestamp::now(clock);

        // First step, just propagate the initial conditions
        if i == 0 {
            self.output.update(
                t,
                &self.state,
                &RocketState::default(),
                &ServoPosition::default(),
                &*self.engine,
                &self.params,
                &self.aerodynamics,
            );
            return Ok(StepResult::Continue);
        }

        let Timestamped(_, servo_pos) = self.rx_servo_pos.recv()?;
        self.step_state.servo_pos = servo_pos.clone();

        let next = RungeKutta4.solve(
            self,
            t.monotonic.elapsed_seconds_f64(),
            TD(dt).seconds(),
            self.state.0,
        );

        self.state.0 = next;

        // Normalize quaternion agains numerical errors
        self.state.normalize_quat();

        self.output.update(
            t,
            &self.state,
            &RocketState(self.odefun(
                t.monotonic.elapsed_seconds_f64(),
                self.state.0.clone_owned(),
            )),
            &servo_pos,
            &*self.engine,
            &self.params,
            &self.aerodynamics,
        );

        // Stop conditions
        if (self.state.pos_n()[2] > 0.0 && t.monotonic.elapsed_seconds_f64() > 1.0)
            || t.monotonic.elapsed_seconds_f64() > self.params.max_t
        {
            Ok(StepResult::Stop)
        } else {
            Ok(StepResult::Continue)
        }
    }
}
