use nalgebra::Vector3;

pub trait RocketEngine {
    /// Thrust of the rocket at time tburn, in the body frame
    fn thrust_b(&self, t: f64) -> Vector3<f64>;
}

