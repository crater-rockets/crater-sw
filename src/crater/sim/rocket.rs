use core::f64;

use crate::{
    core::time::{Clock, Instant, Timestamp, TD},
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
    coefficients: Coefficients,
    state: State,
    senders: Senders,
    dt: Dt,
}

struct Params {
    mass: f64,
    inertia: Matrix3<f64>,
    inv_inertia: Matrix3<f64>,
    p0_n: Vector3<f64>,
    v0_b: Vector3<f64>,
    w0_b: Vector3<f64>,
    g_n: Vector3<f64>,
    diameter: f64,
    surface: f64,
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

        let p0_n = param_service.get_vec_f64(format!("{path}/init/p0_n").as_str())?;
        let p0_n = Vector3::from_column_slice(&p0_n);

        let v0_b = param_service.get_vec_f64(format!("{path}/init/v0_b").as_str())?;
        let v0_b = Vector3::from_column_slice(&v0_b);

        let mut w0_b = param_service.get_vec_f64(format!("{path}/init/w0_b_deg").as_str())?;
        w0_b.iter_mut().for_each(|w| *w = w.to_radians());
        let w0_b = Vector3::from_column_slice(&w0_b);

        let g_n = param_service.get_vec_f64(format!("{path}/g_n").as_str())?;
        let g_n = Vector3::from_column_slice(&g_n);

        Ok(Params {
            mass: param_service.get_f64(format!("{path}/mass").as_str())?,
            inertia,
            inv_inertia,
            p0_n,
            v0_b,
            w0_b,
            g_n,
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

        let mut p_view = state.fixed_rows_mut::<3>(0);
        p_view.set_column(0, &params.p0_n);

        let mut v_view = state.fixed_rows_mut::<3>(3);
        v_view.set_column(0, &q_nb.transform_vector(&params.v0_b));

        let mut q_view = state.fixed_rows_mut::<4>(6);
        q_view.set_column(0, q_nb.as_vector());

        let mut w_view = state.fixed_rows_mut::<3>(10);
        w_view.set_column(0, &params.w0_b);

        Self(state)
    }

    fn pos_n(&self) -> VectorView<'_, f64, U3, U1, U13> {
        self.0.fixed_rows::<3>(0)
    }

    fn vel_n(&self) -> VectorView<'_, f64, U3, U1, U13> {
        self.0.fixed_rows::<3>(3)
    }

    fn quat_nb_vec(&self) -> VectorView<'_, f64, U4, U1, U13> {
        self.0.fixed_rows::<4>(6)
    }

    fn angvel_b(&self) -> VectorView<'_, f64, U3, U1, U13> {
        self.0.fixed_rows::<3>(10)
    }

    fn pos_n_mut(&mut self) -> VectorViewMut<'_, f64, U3, U1, U13> {
        self.0.fixed_rows_mut::<3>(0)
    }
    fn vel_n_mut(&mut self) -> VectorViewMut<'_, f64, U3, U1, U13> {
        self.0.fixed_rows_mut::<3>(3)
    }

    fn quat_nb_vec_mut(&mut self) -> VectorViewMut<'_, f64, U4, U1, U13> {
        self.0.fixed_rows_mut::<4>(6)
    }

    fn angvel_b_mut(&mut self) -> VectorViewMut<'_, f64, U3, U1, U13> {
        self.0.fixed_rows_mut::<3>(10)
    }

    fn quat_nb(&self) -> UnitQuaternion<f64> {
        UnitQuaternion::from_quaternion(Quaternion::from_vector(self.quat_nb_vec().clone_owned()))
    }

    fn normalize_quat(&mut self) {
        let n = self.quat_nb_vec().normalize();
        self.quat_nb_vec_mut().set_column(0, &n);
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
            coefficients: Coefficients::from_params(&param_path, &ctx.parameters())?,
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

        let q_nb = state.quat_nb();
        let vel_b = q_nb.inverse_transform_vector(&state.vel_n().clone_owned());
        let aero = Aerodynamics::new(
            vel_b,
            Vector3::zeros(),
            state.angvel_b().clone_owned(),
            self.params.diameter,
            self.params.surface,
        );

        let (f_aero_b, m_aero_b) = aero.actions(&self.coefficients);

        let f_n = q_nb.transform_vector(&(self.engine.thrust_b(t) + f_aero_b)) + self.params.g_n;
        let m_b = m_aero_b;

        let acc_n = f_n / self.params.mass;

        let w_b = state.angvel_b();
        let qw =
            Quaternion::from_vector(Vector4::new(w_b[0] / 2.0, w_b[1] / 2.0, w_b[2] / 2.0, 0.0));
        let qdot = q_nb.into_inner() * qw;

        let w_dot = self.params.inv_inertia * (m_b + (self.params.inertia * w_b).cross(&w_b));

        dstate
            .pos_n_mut()
            .set_column(0, &state.vel_n().clone_owned());
        dstate.vel_n_mut().set_column(0, &acc_n);
        dstate.quat_nb_vec_mut().set_column(0, qdot.as_vector());
        dstate.angvel_b_mut().set_column(0, &w_dot);

        dstate.0
    }
}

impl Node for Rocket {
    fn step(&mut self, clock: &dyn Clock) -> Result<StepResult> {
        let t = Timestamp::now(clock);

        let dt = self.dt.calc_dt(t.monotonic);

        // First step, just propagate the initial conditions
        if dt.is_none() {
            self.senders.send(
                t,
                &self.state,
                &*self.engine,
                &self.params,
                &self.coefficients,
            );
            return Ok(StepResult::Continue);
        }

        let next = RungeKutta4.solve(
            self,
            t.monotonic.elapsed_seconds_f64(),
            TD(dt.unwrap()).seconds(),
            self.state.0,
        );

        self.state.0 = next;
        self.state.normalize_quat();

        self.senders.send(
            t,
            &self.state,
            &*self.engine,
            &self.params,
            &self.coefficients,
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

struct Senders {
    snd_pos: TelemetrySender<Position>,
    snd_vel_ned: TelemetrySender<Velocity>,
    snd_vel_body: TelemetrySender<Velocity>,
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
            snd_vel_ned: telemetry.publish("/rocket/velocity_ned")?,
            snd_vel_body: telemetry.publish("/rocket/velocity_body")?,
            snd_quat: telemetry.publish("/rocket/orientation/quat")?,
            snd_euler: telemetry.publish("/rocket/orientation/euler")?,
            snd_angvel: telemetry.publish("/rocket/angular_vel")?,
            snd_thrust: telemetry.publish("/rocket/thrust")?,
            snd_aerostate: telemetry.publish("/rocket/aero/state")?,
            snd_aeroforces: telemetry.publish("/rocket/aero/actions")?,
        })
    }

    fn send(
        &self,
        t: Timestamp,
        state: &State,
        engine: &dyn RocketEngine,
        params: &Params,
        coeffs: &Coefficients,
    ) {
        let ts = t.monotonic.elapsed().num_nanoseconds().unwrap();

        let q_nb = state.quat_nb();

        self.snd_pos.send(
            t,
            Position {
                timestamp: ts,
                pos: Vec3::from(state.pos_n()),
            },
        );

        self.snd_vel_ned.send(
            t,
            Velocity {
                timestamp: ts,
                vel: Vec3::from(state.vel_n()),
            },
        );

        self.snd_vel_body.send(
            t,
            Velocity {
                timestamp: ts,
                vel: Vec3::from(q_nb.inverse_transform_vector(&state.vel_n().clone_owned())),
            },
        );

        self.snd_quat.send(
            t,
            OrientationQuat {
                timestamp: ts,
                quat: crate::crater_messages::basic::Quaternion::from(state.quat_nb_vec()),
            },
        );

        let (roll, pitch, yaw) = q_nb.euler_angles();

        self.snd_euler.send(
            t,
            EulerAngles {
                timestamp: ts,
                yaw: yaw.to_degrees(),
                pitch: pitch.to_degrees(),
                roll: roll.to_degrees(),
            },
        );

        let angvel = state.angvel_b();
        self.snd_angvel.send(
            t,
            AngularVelocity {
                timestamp: ts,
                ang_vel: Vec3::from(vector![
                    angvel[0].to_degrees(),
                    angvel[1].to_degrees(),
                    angvel[2].to_degrees()
                ]),
            },
        );

        self.snd_thrust.send(
            t,
            Thrust {
                timestamp: ts,
                thrust: Vec3::from(engine.thrust_b(t.monotonic.elapsed_seconds_f64())),
            },
        );

        let aero = Aerodynamics::new(
            q_nb.inverse_transform_vector(&state.vel_n().clone_owned()),
            Vector3::zeros(),
            state.angvel_b().clone_owned(),
            params.diameter,
            params.surface,
        );

        self.snd_aerostate.send(
            t,
            AeroState {
                timestamp: ts,
                alpha: aero.alpha().to_degrees(),
                beta: aero.beta().to_degrees(),
            },
        );

        let (af, at) = aero.actions(coeffs);

        self.snd_aeroforces.send(
            t,
            AeroForces {
                timestamp: ts,
                force: Vec3::from(af),
                torque: Vec3::from(at),
            },
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{matrix, vector, Matrix, Matrix3, RawStorage, Unit, UnitVector3, U1, U3};

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
        let (yaw, pitch, roll) = (45.0f64, 45.0f64, 0.0f64);

        let q_nb = UnitQuaternion::from_euler_angles(
            roll.to_radians(),
            pitch.to_radians(),
            yaw.to_radians(),
        );

        println!("{}", q_nb.as_vector().clone_owned());
        println!("{}", q_nb.transform_vector(&vector![1.0, 0.0, 0.0]));
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
