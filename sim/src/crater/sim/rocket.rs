use super::{
    aero::aerodynamics::AerodynamicsResult,
    engine::{engine::RocketEngine, SimpleRocketEngine, TabRocketEngine},
    gnc::ServoPosition,
    rocket_data::{RocketMassProperties, RocketParams, RocketState},
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
use anyhow::{anyhow, Ok, Result};
use chrono::TimeDelta;
use core::f64;
use nalgebra::{Quaternion, SVector, UnitQuaternion, Vector3, Vector4};

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
        let rocket_params = ctx.parameters().get_map("sim.rocket")?;

        // Select which engine to use based on the config file (currently only one option)
        let engine: Box<dyn RocketEngine + Send> = match rocket_params
            .get_param("engine.engine_type")?
            .value_string()?
            .as_str()
        {
            "simple" => Box::new(SimpleRocketEngine::from_impulse(
                rocket_params
                    .get_param("engine.simple.total_impulse")?
                    .value_float()?,
                rocket_params
                    .get_param("engine.simple.thrust_duration")?
                    .value_float()?,
            )),
            "tabulated" => Box::new(TabRocketEngine::from_json(
                rocket_params
                    .get_param("engine.tabulated.json_path")?
                    .value_string()?
                    .as_str(),
            )?),
            unknown => {
                return Err(anyhow!(
                    "Unknown engine type selected for rocket '{name}': {unknown}"
                ))
            }
        };

        // Read parameters
        let params = RocketParams::from_params(rocket_params)?;

        // Initialize state with initial conditions from parameters
        let state = RocketState::from_params(&params);

        let atmosphere = Box::new(AtmosphereIsa::default());

        let aero_params = rocket_params.get_map("aero")?;
        let aero_coefficients = AeroCoefficients::from_params(aero_params)?;
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
        let state: RocketState = RocketState(y);

        // state derivative
        let mut dstate: RocketState = RocketState::default();

        let q_nb: UnitQuaternion<f64> = state.quat_nb();
        let vel_b: Vector3<f64> = q_nb.inverse_transform_vector(&state.vel_n().clone_owned());
        let w_b: Vector3<f64> = state.angvel_b();

        let aero: AerodynamicsResult = self.aerodynamics.calc(&AeroState::new(
            self.step_state.servo_pos.mix(),
            vel_b,
            Vector3::zeros(),
            w_b.clone_owned(),
            -state.pos_n()[2],
        ));

        let mass_props = RocketMassProperties::calc_mass(&self.engine.mass(t), &self.params);

        let f_n: Vector3<f64> = q_nb.transform_vector(
            &(self.engine.thrust_b(t) + &aero.forces + &self.params.disturb_const_force_b),
        ) - mass_props.mass_dot * &state.vel_n();

        let m_b: Vector3<f64> = aero.moments + self.params.disturb_const_torque_b;

        let acc_n: Vector3<f64> = (f_n) / mass_props.mass + self.params.g_n;

        let qw: Quaternion<f64> =
            Quaternion::from_vector(Vector4::new(w_b[0] / 2.0, w_b[1] / 2.0, w_b[2] / 2.0, 0.0));
        let qdot: Quaternion<f64> = q_nb.into_inner() * qw;

        let w_dot: Vector3<f64> = mass_props.inertia.try_inverse().unwrap()
            * (m_b - mass_props.inertia_dot * w_b + (mass_props.inertia * w_b).cross(&w_b));

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
                &self.aerodynamics,
                &RocketMassProperties::calc_mass(
                    &self.engine.mass(t.monotonic.elapsed_seconds_f64()),
                    &self.params,
                ),
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
            &self.aerodynamics,
            &RocketMassProperties::calc_mass(
                &self.engine.mass(t.monotonic.elapsed_seconds_f64()),
                &self.params,
            ),
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
