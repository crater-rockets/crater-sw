use core::f64;

use crate::{
    crater::sim::engine::SimpleRocketEngine,
    crater_messages::{
        basic::Vec3,
        sensors::{
            AeroForces, AeroState, AngularVelocity, EulerAngles, OrientationQuat, Position, Thrust,
            Velocity,
        },
    },
    math::ode::{OdeProblem, OdeSolver, RungeKutta4},
    nodes::{Node, NodeContext, NodeTelemetry, StepResult},
    parameters::ParameterService,
    telemetry::{TelemetryDispatcher, TelemetrySender},
    utils::time::{Clock, Instant, TD},
};
use anyhow::{anyhow, Result};
use chrono::TimeDelta;
use nalgebra::{
    vector, Matrix3, Quaternion, SVector, UnitQuaternion, Vector3, Vector4, VectorView,
    VectorViewMut, U1, U13, U3, U4,
};

use super::{
    aerodynamics::{Aerodynamics, Coefficients},
    engine::RocketEngine,
};

pub struct Rocket {
    engine: Box<dyn RocketEngine + Send>,
    params: Params,
    state: State,
    senders: Senders,
    dt: Dt,
}

struct Params {
    mass: f64,
    inertia: Matrix3<f64>,
    diameter: f64,
    surface: f64,
    inv_inertia: Matrix3<f64>,
    max_t: f64,
    azimuth: f64,
    elevation: f64,
}

impl Params {
    fn from_service(path: &str, param_service: &ParameterService) -> Result<Self> {
        let inertia = param_service.get_vec_f64(format!("{path}/inertia").as_str())?;

        let inertia = Matrix3::from_diagonal(&Vector3::from_column_slice(&inertia));
        let inv_inertia = inertia
            .try_inverse()
            .ok_or(anyhow!("The intertia matrix is not invertible"))?;

        let diameter = param_service.get_f64(format!("{path}/diameter").as_str())?;
        let surface = f64::consts::PI * (diameter / 2.0).powf(2.0);

        Ok(Params {
            mass: param_service.get_f64(format!("{path}/mass").as_str())?,
            inertia,
            inv_inertia,
            diameter,
            surface,
            max_t: param_service.get_f64(format!("/sim/max_t").as_str())?,
            azimuth: param_service
                .get_f64(format!("{path}/init/azimuth").as_str())?
                .to_radians(),
            elevation: param_service
                .get_f64(format!("{path}/init/elevation").as_str())?
                .to_radians(),
        })
    }
}

#[derive(Debug, Default)]
struct State(SVector<f64, 13>);

impl State {
    fn from_params(params: &Params) -> Self {
        let q_nb = UnitQuaternion::from_euler_angles(0.0, params.elevation, params.azimuth);

        let mut state: SVector<f64, 13> = SVector::zeros();

        let mut v = state.fixed_rows_mut::<4>(6);
        v.set_column(0, q_nb.as_vector());

        Self(state)
    }

    fn pos(&self) -> VectorView<'_, f64, U3, U1, U13> {
        self.0.fixed_rows::<3>(0)
    }

    fn vel(&self) -> VectorView<'_, f64, U3, U1, U13> {
        self.0.fixed_rows::<3>(3)
    }

    fn quat_vec(&self) -> VectorView<'_, f64, U4, U1, U13> {
        self.0.fixed_rows::<4>(6)
    }

    fn angvel(&self) -> VectorView<'_, f64, U3, U1, U13> {
        self.0.fixed_rows::<3>(10)
    }

    fn pos_mut(&mut self) -> VectorViewMut<'_, f64, U3, U1, U13> {
        self.0.fixed_rows_mut::<3>(0)
    }
    fn vel_mut(&mut self) -> VectorViewMut<'_, f64, U3, U1, U13> {
        self.0.fixed_rows_mut::<3>(3)
    }

    fn quat_vec_mut(&mut self) -> VectorViewMut<'_, f64, U4, U1, U13> {
        self.0.fixed_rows_mut::<4>(6)
    }

    fn angvel_mut(&mut self) -> VectorViewMut<'_, f64, U3, U1, U13> {
        self.0.fixed_rows_mut::<3>(10)
    }

    fn quat(&self) -> UnitQuaternion<f64> {
        UnitQuaternion::from_quaternion(Quaternion::from_vector(self.quat_vec().clone_owned()))
    }

    fn normalize_quat(&mut self) {
        let n = self.quat_vec().normalize();
        self.quat_vec_mut().set_column(0, &n);
    }
}

impl Rocket {
    pub fn new(name: &str, ctx: NodeContext) -> Result<Self> {
        let param_path = format!("/sim/rocket/{name}");

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

        let params = Params::from_service(&param_path, &ctx.parameters())?;
        let senders = Senders::new(ctx.telemetry())?;
        let state = State::from_params(&params);

        Ok(Rocket {
            engine,
            params,
            state,
            senders,
            dt: Dt::new(),
        })
    }
}

impl OdeProblem<f64, 13> for Rocket {
    fn odefun(&self, t: f64, y: SVector<f64, 13>) -> SVector<f64, 13> {
        let state = State(y);
        let mut dstate = State::default();

        let q_nb = state.quat();

        let aero = Aerodynamics::new(
            state.vel().clone_owned(),
            Vector3::zeros(),
            state.angvel().clone_owned(),
            self.params.diameter,
            self.params.surface,
        );

        let (f_aero, m_aero) = aero.actions(&Coefficients::default());

        let f_n =
            q_nb.transform_vector(&(self.engine.thrust(t) + f_aero)) + vector![0.0, 0.0, 9.81];
        let m = m_aero;

        let acc_n = f_n / self.params.mass;
        let vel = state.vel().clone_owned();

        let w = state.angvel();
        let qw = Quaternion::from_vector(Vector4::new(w[0], w[1], w[2], 0.0) / 2.0);
        let qdot = q_nb.into_inner() * qw;

        let w_dot = self.params.inv_inertia * (m + (self.params.inertia * w).cross(&w));

        dstate.pos_mut().set_column(0, &vel);
        dstate.vel_mut().set_column(0, &acc_n);
        dstate.quat_vec_mut().set_column(0, qdot.as_vector());
        dstate.angvel_mut().set_column(0, &w_dot);

        dstate.0
    }
}

impl Node for Rocket {
    fn step(&mut self, clock: &dyn Clock) -> Result<StepResult> {
        let t = clock.monotonic();
        let dt = self.dt.calc_dt(t);

        // First step, just propagate the initial conditions
        if dt.is_none() {
            self.senders
                .send(t, &self.state, &*self.engine, &self.params);
            return Ok(StepResult::Continue);
        }

        let next = RungeKutta4.solve(
            self,
            t.elapsed_seconds(),
            TD(dt.unwrap()).seconds(),
            self.state.0,
        );

        self.state.0 = next;
        self.state.normalize_quat();

        self.senders
            .send(t, &self.state, &*self.engine, &self.params);

        // Stop conditions
        if (self.state.pos()[2] > 0.0 && t.elapsed_seconds() > 1.0)
            || t.elapsed_seconds() > self.params.max_t
        {
            Ok(StepResult::Stop)
        } else {
            Ok(StepResult::Continue)
        }
    }
}

struct Senders {
    snd_pos: TelemetrySender<Position>,
    snd_vel: TelemetrySender<Velocity>,
    snd_quat: TelemetrySender<OrientationQuat>,
    snd_euler: TelemetrySender<EulerAngles>,
    snd_angvel: TelemetrySender<AngularVelocity>,
    snd_thrust: TelemetrySender<Thrust>,
    snd_aerostate: TelemetrySender<AeroState>,
    snd_aeroforces: TelemetrySender<AeroForces>,
}

impl Senders {
    fn new(telemetry: &NodeTelemetry) -> Result<Self> {
        Ok(Self {
            snd_pos: telemetry.publish("/rocket/position")?,
            snd_vel: telemetry.publish("/rocket/velocity")?,
            snd_quat: telemetry.publish("/rocket/orientation/quat")?,
            snd_euler: telemetry.publish("/rocket/orientation/euler")?,
            snd_angvel: telemetry.publish("/rocket/angular_vel")?,
            snd_thrust: telemetry.publish("/rocket/thrust")?,
            snd_aerostate: telemetry.publish("/rocket/aero/state")?,
            snd_aeroforces: telemetry.publish("/rocket/aero/actions")?,
        })
    }

    fn send(&self, t: Instant, state: &State, engine: &dyn RocketEngine, params: &Params) {
        let timestamp = t.elapsed().num_nanoseconds().unwrap();

        self.snd_pos.send(Position {
            timestamp,
            pos: Vec3::from(state.pos()),
        });

        self.snd_vel.send(Velocity {
            timestamp,
            vel: Vec3::from(state.vel()),
        });

        self.snd_quat.send(OrientationQuat {
            timestamp,
            quat: crate::crater_messages::basic::Quaternion::from(state.quat_vec()),
        });

        let (roll, pitch, yaw) = state.quat().euler_angles();

        self.snd_euler.send(EulerAngles {
            timestamp,
            yaw: yaw.to_degrees(),
            pitch: pitch.to_degrees(),
            roll: roll.to_degrees(),
        });

        self.snd_angvel.send(AngularVelocity {
            timestamp,
            ang_vel: Vec3::from(state.angvel()),
        });

        self.snd_thrust.send(Thrust {
            timestamp,
            thrust: Vec3::from(engine.thrust(t.elapsed_seconds())),
        });

        let aero = Aerodynamics::new(
            state.vel().clone_owned(),
            Vector3::zeros(),
            state.angvel().clone_owned(),
            params.diameter,
            params.surface,
        );

        self.snd_aerostate.send(AeroState {
            timestamp,
            alpha: aero.alpha().to_degrees(),
            beta: aero.beta().to_degrees(),
        });

        let (af, at) = aero.actions(&Coefficients::default());

        self.snd_aeroforces.send(AeroForces {
            timestamp,
            force: Vec3::from(af),
            torque: Vec3::from(at),
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{matrix, vector, Matrix, Matrix3, RawStorage, U1, U3};

    #[allow(dead_code)]
    fn cross_product_matrix<S: RawStorage<f64, U3, U1>>(x: Matrix<f64, U3, U1, S>) -> Matrix3<f64> {
        matrix!(
            0.0, -x[2], x[1];
            x[2], 0.0, -x[0];
            -x[1], x[0], 0.0
        )
    }

    #[test]
    fn test_quaternion() {
        let q_nb =
            UnitQuaternion::from_euler_angles(0.0, 45.0f64.to_radians(), 45.0f64.to_radians());

        let x0 = vector![1.0, 0.0, 0.0];

        println!("{}", q_nb.transform_vector(&x0));
    }
}

struct Dt {
    last_ts: Option<Instant>,
}

impl Dt {
    fn new() -> Self {
        Self { last_ts: None }
    }

    fn calc_dt(&mut self, t: Instant) -> Option<TimeDelta> {
        if let Some(last_ts) = self.last_ts {
            let dt = t.duration_since(&last_ts);
            self.last_ts = Some(t);
            Some(dt)
        } else {
            self.last_ts = Some(t);
            None
        }
    }
}
