use core::f64;

use anyhow::{anyhow, Result};
use nalgebra::{matrix, Matrix3, Quaternion, SVector, UnitQuaternion, Vector3, Vector4};

use crate::parameters::ParameterService;

use super::engine::engine::RocketEngineMasses;

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

#[derive(Debug, Clone, Default)]
pub struct RocketActions {
    pub thrust_b: Vector3<f64>,
    pub aero_force_b: Vector3<f64>,
    pub aero_torque_b: Vector3<f64>,

    pub acc_b: Vector3<f64>, // Acceleration
    pub ang_acc: Vector3<f64> // Angular acceleration
}

#[derive(Debug, Clone)]
pub struct AeroAngles {
    pub alpha: f64,
    pub beta: f64,
}

#[derive(Debug, Clone)]
pub struct RocketMassProperties {
    pub xcg_total: Vector3<f64>,
    pub mass_tot: f64,
    pub mass_dot: f64,
    pub inertia: Matrix3<f64>,
    pub inertia_dot: Matrix3<f64>,
}

impl RocketMassProperties {
    pub fn calc_mass(prop_mass: RocketEngineMasses, rocket: RocketParams) -> RocketMassProperties{
        let mass_tot = rocket.mass_empty + prop_mass.mass;

        let mass_dot = prop_mass.mass_dot;

        let xcg_prop = rocket.xcg_motor_ref + Vector3::new(prop_mass.xcg, 0.0, 0.0);

        let xcg_total = (prop_mass.mass * (xcg_prop) + rocket.mass_empty * (rocket.xcg_empty))
            / mass_tot;

        let inertia_empt: Matrix3<f64> = rocket.inertia_empty
            - rocket.mass_empty
                * self::RocketMassProperties::parallel_axis_matrix(
                    xcg_total - rocket.xcg_empty,
                );

        let inertia_mot: Matrix3<f64> = prop_mass.inertia
            - prop_mass.mass
                * self::RocketMassProperties::parallel_axis_matrix(xcg_total - xcg_prop);

        let inertia = inertia_empt + inertia_mot;

        let diff_prop_xcg: Vector3<f64> = xcg_total - xcg_prop;

        let skew_prop_xcg: Matrix3<f64> = self::RocketMassProperties::skew_matrix(diff_prop_xcg);

        let skew_prop_dot_xcg =
            self::RocketMassProperties::skew_matrix(Vector3::new(prop_mass.xcg_dot, 0.0, 0.0));

        let inertia_dot = prop_mass.inertia_dot
            - prop_mass.mass
                * (2.0 * skew_prop_xcg.transpose() * skew_prop_dot_xcg
                    + skew_prop_dot_xcg.transpose() * skew_prop_xcg);

        RocketMassProperties{
            xcg_total,
            mass_tot,
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
    pub mass_empty: f64,
    pub inertia_empty: Matrix3<f64>,
    pub xcg_datcom: Vector3<f64>,
    pub xcg_empty: Vector3<f64>,
    pub xcg_motor_ref: Vector3<f64>,
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
    pub imu_pos: Vector3<f64>,
}

impl RocketParams {
    pub fn from_service(path: &str, param_service: &ParameterService) -> Result<Self> {
        let inertia = param_service.get_vec_f64(format!("{path}/inertia_empty").as_str())?;

        let inertia_empty = Matrix3::from_column_slice(&inertia);

        let xcg_datcom = param_service.get_vec_f64(format!("{path}/xcg_datcom").as_str())?;
        let xcg_datcom = Vector3::from_column_slice(&xcg_datcom);

        let xcg_empty = param_service.get_vec_f64(format!("{path}/xcg_empty").as_str())?;
        let xcg_empty = Vector3::from_column_slice(&xcg_empty);

        let xcg_motor_ref = param_service.get_vec_f64(format!("{path}/xcg_motor_ref").as_str())?;
        let xcg_motor_ref = Vector3::from_column_slice(&xcg_motor_ref);

        let var_name = anyhow!("The intertia matrix is not invertible");
        let inv_inertia = inertia_empty.try_inverse().ok_or(var_name)?;

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

        let imu_pos = param_service.get_vec_f64(format!("{path}/imu/imu_position").as_str())?;
        let imu_pos = Vector3::from_column_slice(&imu_pos);
        
        Ok(RocketParams {
            mass_empty: param_service.get_f64(format!("{path}/mass").as_str())?,
            inertia_empty,
            xcg_datcom,
            xcg_empty,
            xcg_motor_ref,
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
            imu_pos,
        })
    }
}
