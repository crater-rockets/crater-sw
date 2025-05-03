use nalgebra::{Matrix3, Vector3};

#[derive(Debug, Clone)]
pub struct RocketEngineMassProperties {
    pub xcg_eng_frame: f64,
    pub xcg_dot_eng_frame: f64,

    pub mass: f64,
    pub mass_dot: f64,
    
    pub inertia_eng_frame: Matrix3<f64>,
    pub inertia_dot_eng_frame: Matrix3<f64>,
}

pub trait RocketEngine {
    
    /// Thrust of the rocket at time tburn, in the body frame
    fn thrust_b(&self, t_sec: f64) -> Vector3<f64>;

    fn mass(&self, t_sec: f64) -> RocketEngineMassProperties;
}
