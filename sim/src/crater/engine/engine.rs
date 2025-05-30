use nalgebra::{Matrix3, Vector3};

#[derive(Debug, Clone)]
pub struct RocketEngineMassProperties {
    pub xcg_eng_frame_m: f64,
    pub xcg_dot_eng_frame_m: f64,

    pub mass_kg: f64,
    pub mass_dot_kg_s: f64,
    
    pub inertia_eng_frame_kgm2: Matrix3<f64>,
    pub inertia_dot_eng_frame_kgm2: Matrix3<f64>,
}

pub trait RocketEngine {
    
    /// Thrust of the rocket at time tburn, in the body frame
    fn thrust_b(&self, t_sec: f64) -> Vector3<f64>;

    fn mass(&self, t_sec: f64) -> RocketEngineMassProperties;
}
