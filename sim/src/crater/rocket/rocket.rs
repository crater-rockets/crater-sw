use super::{
    mass::RocketMassProperties,
    rocket_data::{RocketAccelerations, RocketActions, RocketParams, RocketState},
    rocket_output::RocketOutput,
};
use crate::{
    core::time::{Clock, TD, Timestamp},
    crater::{
        aero::{
            aerodynamics::{
                AeroCoefficientsValues, AeroState, Aerodynamics, AerodynamicsCoefficients,
            },
            atmosphere::{Atmosphere, AtmosphereIsa, AtmosphereProperties, mach_number},
            linear_aerodynamics::LinearizedAeroCoefficients,
            tabulated_aerodynamics::TabulatedAeroCoefficients,
            wind::WindSample,
        },
        channels,
        engine::{
            SimpleRocketEngine, TabRocketEngine,
            engine::{RocketEngine, RocketEngineMassProperties},
        },
        events::{Event, GncEvent, GncEventItem, SimEvent},
        gnc::ServoPosition,
    },
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
    pub(super) aero_coeffs: Box<dyn AerodynamicsCoefficients + Send>,
    pub(super) aerodynamics: Aerodynamics,
    pub(super) atmosphere: Box<dyn Atmosphere + Send>,
    pub(super) wind_state: WindState,

    pub(super) fsm: StateMachine<RocketFsm>,

    rx_servo_pos: TelemetryReceiver<ServoPosition>,
    rx_sim_event: TelemetryReceiver<SimEvent>,
    rx_wind: TelemetryReceiver<WindSample>,

    output: RocketOutput,
}

/// Variables allowed to change between steps, but not within a step (more precisely, during integration of a single step)
#[derive(Debug, Clone, Default)]
pub(super) struct StepState {
    servo_pos: ServoPosition,
}

#[derive(Debug, Clone, Default)]
pub(super) struct WindState {
    wind: WindSample,
}

impl Rocket {
    pub fn new(name: &str, ctx: NodeContext) -> Result<Self> {
        // Base path for the parameters of this Rocket
        let params_map = ctx.parameters().get_map("sim.rocket")?;

        // Read parameters
        let rocket_params = RocketParams::from_params(params_map)?;
        // Initialize state with initial conditions from parameters
        let state = RocketState::from_params(&rocket_params);

        // Select which engine to use based on the config file (currently only one option)
        let engine: Box<dyn RocketEngine + Send> = match params_map
            .get_param("engine.engine_type")?
            .value_string()?
            .as_str()
        {
            "simple" => Box::new(SimpleRocketEngine::from_impulse(
                params_map
                    .get_param("engine.simple.total_impulse")?
                    .value_float()?,
                params_map
                    .get_param("engine.simple.thrust_duration")?
                    .value_float()?,
            )),
            "tabulated" => Box::new(TabRocketEngine::from_json(
                params_map
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

        let aero_coeffs: Box<dyn AerodynamicsCoefficients + Send> =
            match params_map.get_param("aero.model")?.value_string()?.as_str() {
                "linear" => Box::new(LinearizedAeroCoefficients::from_params(
                    params_map.get_map("sim.rocket.aero.linear")?,
                )?),
                "tabulated" => {
                    let coeffs_main_path = params_map
                        .get_param("aero.tabulated.coeffs_main")?
                        .value_string()?;
                    let coeffs_dynamic_path = params_map
                        .get_param("aero.tabulated.coeffs_dynamic")?
                        .value_string()?;

                    // let aero_params = rocket_params.get_map("aero")?;
                    // let aero_coefficients = AeroCoefficients::from_params(aero_params)?;
                    let file1 = PathBuf::from_str(&coeffs_main_path).unwrap();
                    let file2 = PathBuf::from_str(&coeffs_dynamic_path).unwrap();
                    Box::new(TabulatedAeroCoefficients::from_h5(&file1, &file2)?)
                }
                unknown => {
                    return Err(anyhow!(
                        "Unknown aerodynamics model selected for rocket '{name}': {unknown}"
                    ));
                }
            };

        let atmosphere = Box::new(AtmosphereIsa::default());

        let rx_servo_pos = ctx
            .telemetry()
            .subscribe(channels::actuators::IDEAL_SERVO_POSITION, Unbounded)?;

        let rx_sim_event = ctx
            .telemetry()
            .subscribe_mp(channels::sim::SIM_EVENTS, Unbounded)?;
        let rx_wind = ctx.telemetry().subscribe(channels::sim::WIND, Unbounded)?;

        let tx_gnc_event = ctx.telemetry().publish_mp(channels::gnc::GNC_EVENTS)?;
        let tx_sim_event = ctx.telemetry().publish_mp(channels::sim::SIM_EVENTS)?;

        let fsm = RocketFsm::new(tx_gnc_event, tx_sim_event).state_machine();

        let output = RocketOutput::new(ctx.telemetry())?;

        Ok(Rocket {
            engine,
            aerodynamics: Aerodynamics::new(rocket_params.diameter, rocket_params.surface),
            params: rocket_params,
            aero_coeffs,
            atmosphere,
            state,
            rx_servo_pos,
            rx_sim_event,
            rx_wind,
            fsm,
            output,
            step_state: StepState::default(),
            wind_state: WindState::default(),
        })
    }
}

pub(super) struct RocketOdeStep {
    pub _state: RocketState,
    pub d_state: RocketState,
    pub mass_engine: RocketEngineMassProperties,
    pub mass_rocket: RocketMassProperties,
    pub _atmosphere: AtmosphereProperties,
    pub aero_state: AeroState,
    pub _aero_coeffs: AeroCoefficientsValues,
    pub actions: RocketActions,
    pub accels: RocketAccelerations,
}

impl RocketOdeStep {
    pub fn calc(rocket: &Rocket, t_s: f64, state: RocketState) -> Self {
        let mass_engine = rocket.engine.mass(rocket.fsm.t_from_ignition(t_s));

        let mass_rocket = RocketMassProperties::calc_mass(&mass_engine, &rocket.params);

        let altitude_m = -state.pos_n_m()[2];
        let atmosphere_props = rocket.atmosphere.properties(altitude_m);

        let q_nb: UnitQuaternion<f64> = state.quat_nb();
        let vel_b_m_s: Vector3<f64> =
            q_nb.inverse_transform_vector(&state.vel_n_m_s().clone_owned());
        let vel_norm_m_s = vel_b_m_s.norm();

        let w_b_rad_s: Vector3<f64> = state.angvel_b_rad_s();
        let mach = mach_number(vel_norm_m_s, atmosphere_props.speed_of_sound_m_s);

        let v_air_b_m_s =
            vel_b_m_s - q_nb.inverse_transform_vector(&rocket.wind_state.wind.wind_vel_ned);

        let w_b_air_rad_s = w_b_rad_s + rocket.wind_state.wind.wind_ang_vel;

        let aero_state = AeroState::new(
            v_air_b_m_s,
            w_b_air_rad_s,
            altitude_m,
            mach,
            atmosphere_props.air_density_kg_m3,
            rocket.step_state.servo_pos.clone(),
        );

        let aero_coeffs = rocket.aero_coeffs.coefficients(&aero_state);

        // TODO: Apply forces on correct point, not just COM
        let actions =
            Self::rocket_actions(rocket, t_s, &state, &aero_state, &aero_coeffs, &mass_rocket);

        let qw: Quaternion<f64> = Quaternion::from_vector(Vector4::new(
            w_b_rad_s[0] / 2.0,
            w_b_rad_s[1] / 2.0,
            w_b_rad_s[2] / 2.0,
            0.0,
        ));
        let qdot: Quaternion<f64> = q_nb.into_inner() * qw;

        let acc_n_m_s2 = actions.tot_force_n_n / mass_rocket.mass_kg;

        let ang_acc_b_rad_s2: Vector3<f64> = mass_rocket.inertia_kgm2.try_inverse().unwrap()
            * (actions.tot_moment_b_nm - mass_rocket.inertia_dot_kgm2_s * w_b_rad_s
                + (mass_rocket.inertia_kgm2 * w_b_rad_s).cross(&w_b_rad_s));

        let accels = RocketAccelerations {
            acc_b_m_s2: q_nb.inverse_transform_vector(&acc_n_m_s2),
            acc_n_m_s2,
            ang_acc_b_rad_s2,
        };

        let mut d_state = RocketState::default();
        d_state.set_pos_n_m(&state.vel_n_m_s());
        d_state.set_vel_n_m_s(&accels.acc_n_m_s2);
        d_state.set_quat_nb_vec(qdot.as_vector());
        d_state.set_angvel_b_rad_s(&accels.ang_acc_b_rad_s2);

        RocketOdeStep {
            _state: state,
            d_state,
            mass_engine,
            mass_rocket,
            _atmosphere: atmosphere_props,
            aero_state,
            _aero_coeffs: aero_coeffs,
            actions,
            accels,
        }
    }

    pub fn rocket_actions(
        rocket: &Rocket,
        t: f64,
        rocket_state: &RocketState,
        aero_state: &AeroState,
        aero_coeffs: &AeroCoefficientsValues,
        mass_props: &RocketMassProperties,
    ) -> RocketActions {
        let t_ignition = rocket.fsm.t_from_ignition(t);

        let q_nb: UnitQuaternion<f64> = rocket_state.quat_nb();

        let aero_actions = rocket.aerodynamics.actions(&aero_state, &aero_coeffs);

        let aero_force_b_n = aero_actions.forces_b_n;
        let aero_moment_b_nm = aero_actions.moments_b_nm;

        let thrust_b_n = rocket.engine.thrust_b(t_ignition);

        let force_n: Vector3<f64> = q_nb
            .transform_vector(&(thrust_b_n + aero_force_b_n + rocket.params.disturb_const_force_b))
            - mass_props.mass_dot_kg_s * &rocket_state.vel_n_m_s()
            + rocket.params.g_n * mass_props.mass_kg;

        let (tot_force_n_n, tot_moment_b_nm) = match rocket.fsm.state() {
            State::OnPad {} => (Vector3::<f64>::zeros(), Vector3::<f64>::zeros()),
            State::LiftingOff {} | State::FlyingRamp {} => {
                if force_n[2].abs() > rocket.params.g_n[2].abs() * mass_props.mass_kg {
                    (
                        // Only keep component of acceleration parallel to the ramp
                        rocket.params.ramp_versor.dot(&force_n) * rocket.params.ramp_versor,
                        Vector3::<f64>::zeros(),
                    )
                } else {
                    // Thurst not yet high enough to move
                    (Vector3::<f64>::zeros(), Vector3::<f64>::zeros())
                }
            }
            _ => {
                let torque_b: Vector3<f64> =
                    aero_moment_b_nm + rocket.params.disturb_const_torque_b;
                (force_n, torque_b)
            }
        };

        let tot_force_b_n = q_nb.inverse_transform_vector(&tot_force_n_n);

        RocketActions {
            thrust_b_n,
            aero_actions: aero_actions,
            tot_force_n_n,
            tot_force_b_n,
            tot_moment_b_nm,
        }
    }
}

impl OdeProblem<f64, 13> for Rocket {
    fn odefun(&self, t: f64, y: SVector<f64, 13>) -> SVector<f64, 13> {
        let ode_step = RocketOdeStep::calc(&self, t, RocketState(y));

        ode_step.d_state.0
    }
}

impl Node for Rocket {
    fn step(&mut self, i: usize, dt: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
        let t = Timestamp::now(clock);

        // First step, just propagate the initial conditions
        if i == 0 {
            self.output.update(t, &self);
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

        self.step_state.servo_pos = servo_pos;

        let wind = if let Ok(Timestamped(_, wind)) = self.rx_wind.try_recv() {
            wind
        } else {
            WindSample::default()
        };

        self.wind_state.wind = wind;
        
        let next = RungeKutta4.solve(
            self,
            t.monotonic.elapsed_seconds_f64(),
            TD(dt).seconds(),
            self.state.0,
        );

        self.state.0 = next;

        // Normalize quaternion agains numerical errors
        self.state.normalize_quat();

        self.output.update(t, &self);

        // Stop conditions
        if (self.state.pos_n_m()[2] > 0.0 && t.monotonic.elapsed_seconds_f64() > 1.0)
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
                if context.state.pos_n_m()[2] < -0.2 {
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
                if context.state.pos_n_m()[2] < -2.0 {
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
