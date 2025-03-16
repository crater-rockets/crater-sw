use nalgebra::{Matrix3, Vector3};

#[derive(Debug, Clone)]
pub struct RocketEngineMasses {
    pub xcg: f64,
    pub xcg_dot: f64,
    pub mass: f64,
    pub mass_dot: f64,
    pub inertia: Matrix3<f64>,
    pub inertia_dot: Matrix3<f64>,
}

pub trait RocketEngine {
    
    /// Thrust of the rocket at time tburn, in the body frame
    fn thrust_b(&self, t: f64) -> Vector3<f64>;

    fn masses_prop(&self, t: f64) -> RocketEngineMasses;
}
