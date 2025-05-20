use std::f64;

use nalgebra::Vector3;

use crate::crater::sim::gnc::ServoPosition;

#[derive(Debug, Clone)]
pub struct AerodynamicActions {
    pub forces_b_n: Vector3<f64>,
    pub moments_b_nm: Vector3<f64>,
}

#[derive(Debug, Clone)]
pub struct AeroState {
    pub angles: AeroAngles,

    pub mach: f64,
    pub air_density_kg_m3: f64,

    pub v_air_b_m_s: Vector3<f64>,
    pub v_air_norm_m_s: f64,

    pub w_b_rad_s: Vector3<f64>,

    pub altitude_m: f64,

    pub servo_pos: ServoPosition,
}

impl AeroState {
    pub fn new(
        v_air_b_m_s: Vector3<f64>,
        w_b_rad_s: Vector3<f64>,
        altitude_m: f64,
        mach: f64,
        air_density_kg_m3: f64,
        servo_pos: ServoPosition,
    ) -> AeroState {
        let v_air_norm_m_s = v_air_b_m_s.norm();
        AeroState {
            angles: AeroAngles::new(&v_air_b_m_s, v_air_norm_m_s),
            mach,
            air_density_kg_m3,
            v_air_b_m_s,
            v_air_norm_m_s,
            w_b_rad_s,
            altitude_m,
            servo_pos,
        }
    }
}

pub trait Aerodynamics {
    fn actions(&self, state: &AeroState) -> AerodynamicActions;
}

#[derive(Debug, Clone)]
pub struct AeroAngles {
    /// Angle of attack, calcualted with 'atan(w/u)'
    pub alpha_rad: f64,

    /// Sideslip angle, calculated with 'asin(v/V)'
    pub beta_rad: f64,

    // Alternative value for the sideslip angle, calculated with 'atan(v/u)'
    pub beta_tan_rad: f64,
}

impl AeroAngles {
    const V_SMALL: f64 = 1.0e-6;

    pub fn new(v_air_b: &Vector3<f64>, v_air_norm: f64) -> Self {
        AeroAngles {
            alpha_rad: Self::alpha_rad(&v_air_b),
            beta_rad: Self::beta_rad(&v_air_b, v_air_norm),
            beta_tan_rad: Self::beta_tan_rad(&v_air_b),
        }
    }

    pub fn alpha_rad(v_air_b: &Vector3<f64>) -> f64 {
        if v_air_b[0].abs() >= Self::V_SMALL {
            f64::atan(v_air_b[2] / v_air_b[0])
        } else if v_air_b[2].abs() >= Self::V_SMALL {
            f64::consts::FRAC_PI_2 * v_air_b[2].signum()
        } else {
            0.0
        }
    }

    pub fn beta_rad(v_air_b: &Vector3<f64>, v_air_norm: f64) -> f64 {
        if v_air_norm >= Self::V_SMALL {
            f64::asin(v_air_b[1] / v_air_norm)
        } else if v_air_b[1].abs() >= Self::V_SMALL {
            f64::consts::FRAC_PI_2 * v_air_b[1].signum()
        } else {
            0.0
        }
    }

    pub fn beta_tan_rad(v_air_b: &Vector3<f64>) -> f64 {
        if v_air_b[0].abs() >= Self::V_SMALL {
            f64::atan(v_air_b[1] / v_air_b[0])
        } else if v_air_b[1].abs() >= Self::V_SMALL {
            f64::consts::FRAC_PI_2 * v_air_b[1].signum()
        } else {
            0.0
        }
    }
}
