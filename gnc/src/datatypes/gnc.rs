use nalgebra::{UnitQuaternion, Vector3};

#[derive(Debug, Clone)]
pub struct NavigationOutput {
    pub quat_nb: UnitQuaternion<f32>,

    pub pos_n_m: Vector3<f32>,
    pub vel_n_m_s: Vector3<f32>,

    pub angvel_unbias_b_rad_s: Vector3<f32>,
    pub acc_unbias_b_m_s2: Vector3<f32>,
}
