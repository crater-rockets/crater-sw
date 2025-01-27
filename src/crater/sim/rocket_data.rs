use core::f64;

use anyhow::{anyhow, Result};
use nalgebra::{
    Matrix3, Quaternion, SVector, UnitQuaternion, Vector3, Vector4, VectorView, VectorViewMut, U1,
    U13, U3, U4,
};

use crate::parameters::ParameterService;

#[derive(Debug, Default, Clone)]
pub struct RocketState(pub SVector<f64, 13>);

impl RocketState {
    pub fn from_params(params: &RocketParams) -> Self {
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

    pub fn pos_n(&self) -> Vector3<f64> {
        self.0.fixed_rows::<3>(0).clone_owned()
    }

    pub fn vel_n(&self) -> Vector3<f64> {
        self.0.fixed_rows::<3>(3).clone_owned()
    }

    pub fn vel_b(&self) -> Vector3<f64> {
        self.quat_nb()
            .inverse_transform_vector(&self.vel_n().clone_owned())
    }

    pub fn quat_nb_vec(&self) -> Vector4<f64> {
        self.0.fixed_rows::<4>(6).clone_owned()
    }

    pub fn angvel_b(&self) -> Vector3<f64> {
        self.0.fixed_rows::<3>(10).clone_owned()
    }

    pub fn set_pos_n(&mut self, pos_n: &Vector3<f64>) {
        self.0.fixed_rows_mut::<3>(0).set_column(0, pos_n);
    }

    pub fn set_vel_n(&mut self, vel_n: &Vector3<f64>) {
        self.0.fixed_rows_mut::<3>(3).set_column(0, vel_n);
    }

    pub fn set_quat_nb_vec(&mut self, quat_nb: &Vector4<f64>) {
        self.0.fixed_rows_mut::<4>(6).set_column(0, quat_nb);
    }

    pub fn set_angvel_b(&mut self, angvel_b: &Vector3<f64>) {
        self.0.fixed_rows_mut::<3>(10).set_column(0, angvel_b);
    }

    pub fn quat_nb(&self) -> UnitQuaternion<f64> {
        UnitQuaternion::from_quaternion(Quaternion::from_vector(self.quat_nb_vec().clone_owned()))
    }

    pub fn normalize_quat(&mut self) {
        let n = self.quat_nb_vec().normalize();
        self.set_quat_nb_vec(&n);
    }
}

#[derive(Debug, Clone)]
pub struct RocketActions {
    pub thrust_b: Vector3<f64>,
    pub aero_force_b: Vector3<f64>,
    pub aero_torque_b: Vector3<f64>,
}

#[derive(Debug, Clone)]
pub struct AeroAngles {
    pub alpha: f64,
    pub beta: f64,
}

pub struct RocketParams {
    pub mass: f64,
    pub inertia: Matrix3<f64>,
    pub inv_inertia: Matrix3<f64>,
    pub p0_n: Vector3<f64>,
    pub v0_b: Vector3<f64>,
    pub w0_b: Vector3<f64>,
    pub g_n: Vector3<f64>,
    pub diameter: f64,
    pub surface: f64,
    pub max_t: f64,
    pub azimuth: f64,
    pub elevation: f64,
}

impl RocketParams {
    pub fn from_service(path: &str, param_service: &ParameterService) -> Result<Self> {
        let inertia = param_service.get_vec_f64(format!("{path}/inertia").as_str())?;

        let inertia = Matrix3::from_diagonal(&Vector3::from_column_slice(&inertia));
        let var_name = anyhow!("The intertia matrix is not invertible");
        let inv_inertia = inertia.try_inverse().ok_or(var_name)?;

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

        Ok(RocketParams {
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
