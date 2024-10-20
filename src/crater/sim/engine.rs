use nalgebra::Vector3;

pub trait RocketEngine {
    /// Thrust of the rocket at time tburn, in the body frame
    fn thrust(&self, t: f64) -> Vector3<f64>;
}

pub struct SimpleRocketEngine {
    duration: f64,
    thrust: f64,
}

impl SimpleRocketEngine {
    pub fn from_impulse(total_impulse: f64, duration: f64) -> Self {
        SimpleRocketEngine {
            duration,
            thrust: total_impulse / duration,
        }
    }

    pub fn from_thrust(thrust: f64, duration: f64) -> Self {
        SimpleRocketEngine { duration, thrust }
    }
}

impl RocketEngine for SimpleRocketEngine {
    fn thrust(&self, t: f64) -> Vector3<f64> {
        if t >= 0.0 && t <= self.duration {
            Vector3::new(self.thrust, 0.0, 0.0)
        }else{
            Vector3::zeros()
        }
    }
}
