use core::f64;

use anyhow::{anyhow, Result};
use nalgebra::{matrix, Matrix3, Quaternion, SVector, UnitQuaternion, Vector3, Vector4};

use crate::parameters::ParameterMap;

use super::engine::engine::RocketEngineMassProperties;

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

    pub fn vel_b(&self, quat_nb: &UnitQuaternion<f64>) -> Vector3<f64> {
        quat_nb.inverse_transform_vector(&self.vel_n().clone_owned())
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

#[derive(Debug, Clone, Default)]
pub struct RocketActions {
    pub thrust_b: Vector3<f64>,
    pub aero_force_b: Vector3<f64>,
    pub aero_torque_b: Vector3<f64>,

    pub acc_n: Vector3<f64>, // Acceleration
    pub acc_b: Vector3<f64>, // Acceleration
    pub ang_acc_b: Vector3<f64> // Angular acceleration
}

#[derive(Debug, Clone)]
pub struct AeroAngles {
    pub alpha: f64,
    pub beta: f64,
}

#[derive(Debug, Clone)]
pub struct RocketMassProperties {
    pub xcg_total: Vector3<f64>,

    pub mass: f64,
    pub mass_dot: f64,

    pub inertia: Matrix3<f64>,
    pub inertia_dot: Matrix3<f64>,
}

impl RocketMassProperties {
    pub fn calc_mass(mass_eng: &RocketEngineMassProperties, rocket: &RocketParams) -> RocketMassProperties{
        let mass_tot = rocket.mass_body + mass_eng.mass;

        let mass_dot = mass_eng.mass_dot;

        let xcg_eng = rocket.engine_ref_pos + Vector3::new(mass_eng.xcg_eng_frame, 0.0, 0.0);

        let xcg_total = (mass_eng.mass * xcg_eng + rocket.mass_body * (rocket.xcg_body))
            / mass_tot;

        let inertia_body: Matrix3<f64> = rocket.inertia_body_body_frame
            + rocket.mass_body
                * self::RocketMassProperties::parallel_axis_matrix(
                    xcg_total - rocket.xcg_body,
                );

        let parallel_axis_matrix_eng = self::RocketMassProperties::parallel_axis_matrix(xcg_total - xcg_eng);

        let inertia_eng: Matrix3<f64> = mass_eng.inertia_eng_frame
            + mass_eng.mass
                * parallel_axis_matrix_eng;

        let inertia = inertia_body + inertia_eng;

        let dist_prop_xcg: Vector3<f64> = xcg_total - xcg_eng;

        let skew_dist_prop_xcg: Matrix3<f64> = self::RocketMassProperties::skew_matrix(dist_prop_xcg);

        let skew_prop_dot_xcg =
            self::RocketMassProperties::skew_matrix(Vector3::new(mass_eng.xcg_dot_eng_frame, 0.0, 0.0));

        let inertia_dot = mass_eng.inertia_dot_eng_frame
            + mass_eng.mass
                * (skew_dist_prop_xcg.transpose() * skew_prop_dot_xcg
                    + skew_prop_dot_xcg.transpose() * skew_dist_prop_xcg)
                    + mass_dot * parallel_axis_matrix_eng;

        RocketMassProperties{
            xcg_total,
            mass: mass_tot,
            mass_dot,
            inertia,
            inertia_dot
        }
    }

    pub fn skew_matrix(vec: Vector3<f64>) -> Matrix3<f64> {
        matrix![0.0, -vec.z, vec.y; 
                vec.z, 0.0, -vec.x; 
                -vec.y, vec.x, 0.0]
    }
    pub fn parallel_axis_matrix(delta_xcg: Vector3<f64>) -> Matrix3<f64> {
        let cross: Matrix3<f64> = self::RocketMassProperties::skew_matrix(delta_xcg);

        cross.transpose() * cross
    }
}

#[derive(Debug, Clone)]
pub struct RocketParams {
    pub mass_body: f64,
    pub inertia_body_body_frame: Matrix3<f64>,
    pub datcom_ref_pos: Vector3<f64>,
    pub xcg_body: Vector3<f64>,
    pub engine_ref_pos: Vector3<f64>,
    pub inv_inertia: Matrix3<f64>,
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

        let var_name = anyhow!("The intertia matrix is not invertible");
        let inv_inertia = inertia_empty.try_inverse().ok_or(var_name)?;

        let diameter = params.get_param("diameter")?.value_randfloat()?.sampled();
        let surface = f64::consts::PI * (diameter / 2.0).powf(2.0);

        let orig_lat = params.get_param("init.latitude")?.value_float()?.to_radians();
        let orig_lon = params.get_param("init.longitude")?.value_float()?.to_radians();
        let orig_alt = params.get_param("init.altitude")?.value_float()?;

        let origin_geo = Vector3::new(orig_lat,orig_lon,orig_alt);

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

        Ok(RocketParams {
            mass_body: params.get_param("mass")?.value_randfloat()?.sampled(),
            inertia_body_body_frame: inertia_empty,
            datcom_ref_pos,
            xcg_body,
            engine_ref_pos,
            inv_inertia,
            origin_geo,
            p0_n,
            v0_b,
            w0_b,
            g_n,
            diameter,
            surface,
            max_t: params.get_param("max_t")?.value_float()?,
            azimuth: params
                .get_param("init.azimuth")?
                .value_randfloat()?
                .sampled()
                .to_radians(),
            elevation: params
                .get_param("init.elevation")?
                .value_randfloat()?
                .sampled()
                .to_radians(),
            disturb_const_force_b,
            disturb_const_torque_b
        },)
    }
}
