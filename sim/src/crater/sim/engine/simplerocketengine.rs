use super::engine::{RocketEngine,RocketEngineMassProperties};
use nalgebra::{Vector3,Matrix3};

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
    fn thrust_b(&self, t: f64) -> Vector3<f64> {
        if t >= 0.0 && t <= self.duration {
            Vector3::new(self.thrust, 0.0, 0.0)
        } else {
            Vector3::zeros()
        }
    }

    fn mass(&self, t: f64) -> RocketEngineMassProperties {
        let _ = t;
        RocketEngineMassProperties {
            xcg_eng_frame: 0.0,
            xcg_dot_eng_frame: 0.0,
            mass: 0.0,
            mass_dot: 0.0,
            inertia_eng_frame: Matrix3::zeros(),
            inertia_dot_eng_frame: Matrix3::zeros(),
        }
    }
}
