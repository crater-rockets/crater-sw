use core::f64;

use anyhow::Result;
use nalgebra::{Matrix3, Quaternion, SVector, UnitQuaternion, Vector3, Vector4, vector};

use crate::{crater::sim::aero::aerodynamics::AerodynamicActions, parameters::ParameterMap};

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

    pub fn pos_n_m(&self) -> Vector3<f64> {
        self.0.fixed_rows::<3>(0).clone_owned()
    }

    pub fn vel_n_m_s(&self) -> Vector3<f64> {
        self.0.fixed_rows::<3>(3).clone_owned()
    }

    pub fn vel_b_m_s(&self, quat_nb: &UnitQuaternion<f64>) -> Vector3<f64> {
        quat_nb.inverse_transform_vector(&self.vel_n_m_s().clone_owned())
    }

    pub fn quat_nb_vec(&self) -> Vector4<f64> {
        self.0.fixed_rows::<4>(6).clone_owned()
    }

    pub fn angvel_b_rad_s(&self) -> Vector3<f64> {
        self.0.fixed_rows::<3>(10).clone_owned()
    }

    pub fn set_pos_n_m(&mut self, pos_n: &Vector3<f64>) {
        self.0.fixed_rows_mut::<3>(0).set_column(0, pos_n);
    }

    pub fn set_vel_n_m_s(&mut self, vel_n: &Vector3<f64>) {
        self.0.fixed_rows_mut::<3>(3).set_column(0, vel_n);
    }

    pub fn set_quat_nb_vec(&mut self, quat_nb: &Vector4<f64>) {
        self.0.fixed_rows_mut::<4>(6).set_column(0, quat_nb);
    }

    pub fn set_angvel_b_rad_s(&mut self, angvel_b: &Vector3<f64>) {
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
    pub thrust_b_n: Vector3<f64>,

    pub aero_actions: AerodynamicActions,

    pub tot_force_n_n: Vector3<f64>,
    pub tot_force_b_n: Vector3<f64>,
    pub tot_moment_b_nm: Vector3<f64>,
}

#[derive(Debug, Clone, Default)]
pub struct RocketAccelerations {
    pub acc_n_m_s2: Vector3<f64>,       // Acceleration
    pub acc_b_m_s2: Vector3<f64>,       // Acceleration
    pub ang_acc_b_rad_s2: Vector3<f64>, // Angular acceleration
}

#[derive(Debug, Clone)]
pub struct RocketParams {
    pub mass_body_kg: f64,
    pub inertia_body_b_kgm2: Matrix3<f64>,
    pub datcom_ref_pos_m: Vector3<f64>,
    pub xcg_body_m: Vector3<f64>,
    pub engine_ref_pos_m: Vector3<f64>,
    pub origin_geo: Vector3<f64>,
    pub p0_n: Vector3<f64>,
    pub v0_b: Vector3<f64>,
    pub w0_b: Vector3<f64>,
    pub g_n: Vector3<f64>,
    pub diameter: f64,
    pub surface: f64,
    pub max_t: f64,
    pub azimuth: f64,
    pub elevation: f64,
    pub ramp_versor: Vector3<f64>,

    pub disturb_const_force_b: Vector3<f64>,
    pub disturb_const_torque_b: Vector3<f64>,
}

impl RocketParams {
    pub fn from_params(params: &ParameterMap) -> Result<Self> {
        let inertia = params.get_param("inertia_empty")?.value_float_arr()?;

        let inertia_empty = Matrix3::from_column_slice(&inertia);

        let datcom_ref_pos = params.get_param("datcom_ref_pos")?.value_float_arr()?;
        let datcom_ref_pos = Vector3::from_column_slice(&datcom_ref_pos);

        let xcg_body = params.get_param("xcg_body")?.value_float_arr()?;
        let xcg_body = Vector3::from_column_slice(&xcg_body);

        let engine_ref_pos = params.get_param("engine_ref_pos")?.value_float_arr()?;
        let engine_ref_pos = Vector3::from_column_slice(&engine_ref_pos);

        let diameter = params.get_param("diameter")?.value_randfloat()?.sampled();
        let surface = f64::consts::PI * (diameter / 2.0).powf(2.0);

        let orig_lat = params
            .get_param("init.latitude")?
            .value_float()?
            .to_radians();
        let orig_lon = params
            .get_param("init.longitude")?
            .value_float()?
            .to_radians();
        let orig_alt = params.get_param("init.altitude")?.value_float()?;

        let origin_geo = Vector3::new(orig_lat, orig_lon, orig_alt);

        let p0_n = params.get_param("init.p0_n")?.value_float_arr()?;
        let p0_n = Vector3::from_column_slice(&p0_n);

        let v0_b = params.get_param("init.v0_b")?.value_float_arr()?;
        let v0_b = Vector3::from_column_slice(&v0_b);

        let mut w0_b = params
            .get_param("init.w0_b_deg")?
            .value_float_arr()?
            .to_owned();
        w0_b.iter_mut().for_each(|w| *w = w.to_radians());

        let w0_b = Vector3::from_column_slice(&w0_b);

        let g_n = params.get_param("g_n")?.value_float_arr()?;
        let g_n = Vector3::from_column_slice(&g_n);

        let disturb_const_force_b = params
            .get_param("disturbances.const_force_b")?
            .value_float_arr()?;
        let disturb_const_force_b = Vector3::from_column_slice(&disturb_const_force_b);

        let disturb_const_torque_b = params
            .get_param("disturbances.const_torque_b")?
            .value_float_arr()?;
        let disturb_const_torque_b = Vector3::from_column_slice(&disturb_const_torque_b);

        let azimuth = params
            .get_param("init.azimuth")?
            .value_randfloat()?
            .sampled()
            .to_radians();

        let elevation = params
            .get_param("init.elevation")?
            .value_randfloat()?
            .sampled()
            .to_radians();

        let q_nb = UnitQuaternion::from_euler_angles(0.0, elevation, azimuth);
        let mut pad_versor_n = q_nb.transform_vector(&vector![1.0, 0.0, 0.0]);
        pad_versor_n.normalize_mut();

        Ok(RocketParams {
            mass_body_kg: params.get_param("mass")?.value_randfloat()?.sampled(),
            inertia_body_b_kgm2: inertia_empty,
            datcom_ref_pos_m: datcom_ref_pos,
            xcg_body_m: xcg_body,
            engine_ref_pos_m: engine_ref_pos,
            origin_geo,
            p0_n,
            v0_b,
            w0_b,
            g_n,
            diameter,
            surface,
            max_t: params.get_param("max_t")?.value_float()?,
            azimuth,
            elevation,
            ramp_versor: pad_versor_n,
            disturb_const_force_b,
            disturb_const_torque_b,
        })
    }
}
