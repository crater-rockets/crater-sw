use super::{
    aero::{
        aerodynamics::AerodynamicsResult,
        tabulated_aerodynamics::{AeroState, TabulatedAerodynamics},
    },
    engine::{SimpleRocketEngine, TabRocketEngine, engine::RocketEngine},
    events::{Event, GncEvent, GncEventItem, SimEvent},
    gnc::ServoPosition,
    rocket_data::{RocketActions, RocketMassProperties, RocketParams, RocketState},
    rocket_output::RocketOutput,
};
use crate::{
    core::time::{Clock, TD, Timestamp},
    crater::sim::aero::atmosphere::AtmosphereIsa,
    math::ode::{OdeProblem, OdeSolver, RungeKutta4},
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryReceiver, TelemetrySender, Timestamped},
    utils::capacity::Capacity::Unbounded,
};
use anyhow::{Result, anyhow};
use chrono::TimeDelta;
use core::f64;
use crater_gnc::mav_crater::ComponentId;
use nalgebra::{Quaternion, SVector, UnitQuaternion, Vector3, Vector4};
use statig::prelude::*;
use std::{path::PathBuf, str::FromStr};
use strum::AsRefStr;

pub struct Rocket {
    pub(super) params: RocketParams,
    pub(super) state: RocketState,
    pub(super) step_state: StepState,

    pub(super) engine: Box<dyn RocketEngine + Send>,
    pub(super) aerodynamics: TabulatedAerodynamics,

    pub(super) fsm: StateMachine<RocketFsm>,

    rx_servo_pos: TelemetryReceiver<ServoPosition>,
    rx_sim_event: TelemetryReceiver<SimEvent>,

    output: RocketOutput,
}

/// Variables allowed to change between steps, but not within a step (more precisely, during integration of a single step)
#[derive(Debug, Clone, Default)]
pub(super) struct StepState {
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
                ));
            }
        };

        
        // Read parameters
        let params = RocketParams::from_params(rocket_params)?;
        // Initialize state with initial conditions from parameters
        let state = RocketState::from_params(&params);

        let coeffs_main_path = rocket_params.get_param("aero.tabulated.coeffs_main")?.value_string()?;
        let coeffs_dynamic_path = rocket_params.get_param("aero.tabulated.coeffs_dynamic")?.value_string()?;

        let atmosphere = Box::new(AtmosphereIsa::default());

        // let aero_params = rocket_params.get_map("aero")?;
        // let aero_coefficients = AeroCoefficients::from_params(aero_params)?;
        let file1 =
            PathBuf::from_str(&coeffs_main_path).unwrap();
        let file2 =
            PathBuf::from_str(&coeffs_dynamic_path).unwrap();
        let aerodynamics =
            TabulatedAerodynamics::from_h5(&file1, &file2, params.diameter, params.surface)?;

        let rx_servo_pos = ctx
            .telemetry()
            .subscribe("/actuators/servo_position", Unbounded)?;

        let rx_sim_event = ctx.telemetry().subscribe_mp("/sim/events", Unbounded)?;
        let tx_gnc_event = ctx.telemetry().publish_mp("/gnc/events")?;
        let tx_sim_event = ctx.telemetry().publish_mp("/sim/events")?;

        let fsm = RocketFsm::new(tx_gnc_event, tx_sim_event).state_machine();

        let output = RocketOutput::new(ctx.telemetry())?;

        Ok(Rocket {
            engine,
            params,
            aerodynamics,
            state,
            rx_servo_pos,
            rx_sim_event,
            fsm,
            output,
            step_state: StepState::default(),
        })
    }

    pub fn calc_actions(
        &self,
        t: f64,
        state: &RocketState,
        aero: &AerodynamicsResult,
        mass_props: &RocketMassProperties,
    ) -> RocketActions {
        let t_ignition = self.fsm.t_from_ignition(t);

        let q_nb: UnitQuaternion<f64> = state.quat_nb();

        let aero_force_b = aero.forces;
        let aero_torque_b = aero.moments;

        let thrust_b = self.engine.thrust_b(t_ignition);

        let force_n: Vector3<f64> = q_nb
            .transform_vector(&(thrust_b + aero_force_b + self.params.disturb_const_force_b))
            - mass_props.mass_dot * &state.vel_n()
            + self.params.g_n * mass_props.mass;

        let (force_n, torque_b) = match self.fsm.state() {
            State::OnPad {} => (Vector3::<f64>::zeros(), Vector3::<f64>::zeros()),
            State::LiftingOff {} | State::FlyingRamp {} => {
                if force_n[2].abs() > self.params.g_n[2].abs() * mass_props.mass {
                    (
                        // Only keep component of acceleration parallel to the ramp
                        self.params.ramp_versor.dot(&force_n) * self.params.ramp_versor,
                        Vector3::<f64>::zeros(),
                    )
                } else {
                    // Thurst not yet high enough to move
                    (Vector3::<f64>::zeros(), Vector3::<f64>::zeros())
                }
            }
            _ => {
                let torque_b: Vector3<f64> = aero.moments + self.params.disturb_const_torque_b;
                (force_n, torque_b)
            }
        };

        let force_b = q_nb.inverse_transform_vector(&force_n);

        RocketActions {
            thrust_b,
            aero_force_b,
            aero_torque_b,
            force_n,
            force_b,
            torque_b,
        }
    }
}

impl OdeProblem<f64, 13> for Rocket {
    fn odefun(&self, t: f64, y: SVector<f64, 13>) -> SVector<f64, 13> {
        let state: RocketState = RocketState(y);

        // state derivative
        let mut dstate: RocketState = RocketState::default();

        let mass_props = RocketMassProperties::calc_mass(
            &self.engine.mass(self.fsm.t_from_ignition(t)),
            &self.params,
        );

        let q_nb: UnitQuaternion<f64> = state.quat_nb();
        let vel_b: Vector3<f64> = q_nb.inverse_transform_vector(&state.vel_n().clone_owned());
        let w_b: Vector3<f64> = state.angvel_b();

        let mach = vel_b.norm() / 330.0;
        let aerostate = AeroState::new(
            vel_b,
            w_b,
            -state.pos_n()[2],
            mach,
            1.0,
            self.step_state.servo_pos.clone(),
        );

        let (f_aero, m_aero) = self.aerodynamics.actions(&aerostate);

        let aero: AerodynamicsResult = AerodynamicsResult {
            alpha: aerostate.alpha,
            beta: aerostate.beta,
            forces: f_aero,
            moments: m_aero,
        };

        let actions = self.calc_actions(t, &state, &aero, &mass_props);

        let qw: Quaternion<f64> =
            Quaternion::from_vector(Vector4::new(w_b[0] / 2.0, w_b[1] / 2.0, w_b[2] / 2.0, 0.0));
        let qdot: Quaternion<f64> = q_nb.into_inner() * qw;

        let acc_n = actions.force_n / mass_props.mass;
        let w_dot: Vector3<f64> = mass_props.inertia.try_inverse().unwrap()
            * (actions.torque_b - mass_props.inertia_dot * w_b
                + (mass_props.inertia * w_b).cross(&w_b));

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
                &self,
            );

            return Ok(StepResult::Continue);
        }

        let mut fsm_ctx = RocketFsmContext {
            time: Timestamp::now(clock),
            state: self.state.clone(),
        };

        while let Ok(ev) = self.rx_sim_event.try_recv() {
            self.fsm
                .handle_with_context(&Event::Sim(ev.1), &mut fsm_ctx);
        }
        self.fsm.handle_with_context(&Event::Step, &mut fsm_ctx);

        let servo_pos = if let Ok(Timestamped(_, servo_pos)) = self.rx_servo_pos.try_recv() {
            servo_pos
        } else {
            ServoPosition::default()
        };

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
            &RocketState(self.odefun(t.monotonic.elapsed_seconds_f64(), self.state.0.clone())),
            &ServoPosition::default(),
            &self,
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

pub struct RocketFsm {
    tx_gnc_event: TelemetrySender<GncEventItem>,
    tx_sim_event: TelemetrySender<SimEvent>,
    ignition_time: Option<Timestamp>,
}

pub struct RocketFsmContext {
    time: Timestamp,

    state: RocketState,
}

impl RocketFsm {
    fn new(
        tx_gnc_event: TelemetrySender<GncEventItem>,
        tx_sim_event: TelemetrySender<SimEvent>,
    ) -> Self {
        RocketFsm {
            tx_gnc_event,
            tx_sim_event,
            ignition_time: None,
        }
    }

    pub fn t_from_ignition(&self, t: f64) -> f64 {
        if let Some(ignition_time) = self.ignition_time {
            t - ignition_time.monotonic.elapsed_seconds_f64()
        } else {
            0.0
        }
    }
}

#[state_machine(
    initial = "State::on_pad()",
    state(derive(Debug, AsRefStr)),
    superstate(derive(Debug)),
    after_transition = "Self::after_transition"
)]
impl RocketFsm {
    #[state]
    fn on_pad(event: &Event) -> Response<State> {
        match event {
            Event::Sim(SimEvent::StartEngine) => Transition(State::lifting_off()),
            _ => Super,
        }
    }

    #[action]
    fn enter_lifting_off(&mut self, context: &mut RocketFsmContext) {
        self.ignition_time = Some(context.time);
    }

    #[state(entry_action = "enter_lifting_off")]
    fn lifting_off(context: &mut RocketFsmContext, event: &Event) -> Response<State> {
        match event {
            Event::Step => {
                if context.state.pos_n()[2] < -0.2 {
                    Transition(State::flying_ramp())
                } else {
                    Handled
                }
            }
            _ => Super,
        }
    }

    #[action]
    fn enter_flying_ramp(&mut self, context: &RocketFsmContext) {
        self.tx_gnc_event.send(
            context.time,
            GncEventItem {
                src: ComponentId::Ground,
                event: GncEvent::CmdFmmForceLiftoff,
            },
        );
    }

    #[state(entry_action = "enter_flying_ramp")]
    fn flying_ramp(context: &mut RocketFsmContext, event: &Event) -> Response<State> {
        match event {
            Event::Step => {
                // TODO: Better ramp exit condition check
                if context.state.pos_n()[2] < -2.0 {
                    Transition(State::flying_free())
                } else {
                    Handled
                }
            }
            _ => Super,
        }
    }

    #[state]
    fn flying_free(event: &Event) -> Response<State> {
        match event {
            _ => Super,
        }
    }
}

impl RocketFsm {
    fn after_transition(&mut self, source: &State, target: &State, context: &mut RocketFsmContext) {
        self.tx_sim_event.send(
            context.time,
            SimEvent::FsmTransition {
                fsm: "rocket".to_string(),
                source: source.as_ref().to_string(),
                target: target.as_ref().to_string(),
            },
        );
    }
}
