use std::f64;

use nalgebra::{Vector3, vector};

use crate::crater::gnc::ServoPosition;

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

#[allow(nonstandard_style)]
#[derive(Debug, Clone)]
pub struct AeroCoefficientsValues {
    pub cA: f64,

    pub cY: f64,
    pub cY_r: f64,
    pub cY_bd: f64,

    pub cN: f64,
    pub cN_q: f64,
    pub cN_ad: f64,

    pub cl: f64,
    pub cl_p: f64,
    pub cl_r: f64,

    pub cm: f64,
    pub cm_q: f64,
    pub cm_ad: f64,

    pub cn: f64,
    pub cn_r: f64,
    pub cn_bd: f64,
}

pub trait AerodynamicsCoefficients {
    fn coefficients(&self, state: &AeroState) -> AeroCoefficientsValues;
}

pub struct Aerodynamics {
    ref_length_m: f64,
    ref_surface_m2: f64,
}

impl Aerodynamics {
    pub fn new(ref_length_m: f64, ref_surface_m2: f64) -> Self {
        Self {
            ref_length_m,
            ref_surface_m2,
        }
    }

    pub fn actions(&self, state: &AeroState, c: &AeroCoefficientsValues) -> AerodynamicActions {
        let q_v = 0.5 * state.air_density_kg_m3 * state.v_air_norm_m_s;

        let fx = -q_v * self.ref_surface_m2 * (state.v_air_norm_m_s * c.cA);
        let fy = q_v
            * self.ref_surface_m2
            * (state.v_air_norm_m_s * c.cY
                + self.ref_length_m / 2.0 * (c.cY_r + c.cY_bd) * state.w_b_rad_s[2]);
        let fz = -q_v
            * self.ref_surface_m2
            * (state.v_air_norm_m_s * c.cN
                + self.ref_length_m / 2.0 * (c.cN_q + c.cN_ad) * state.w_b_rad_s[2]);

        let mx = q_v
            * self.ref_surface_m2
            * self.ref_length_m
            * (state.v_air_norm_m_s * c.cl
                + self.ref_length_m / 2.0
                    * (c.cl_p * state.w_b_rad_s[0] + c.cl_r * state.w_b_rad_s[2]));

        let my = q_v
            * self.ref_surface_m2
            * self.ref_length_m
            * (state.v_air_norm_m_s * c.cm
                + self.ref_length_m / 2.0 * (c.cm_q + c.cm_ad) * state.w_b_rad_s[1]);

        let mz = q_v
            * self.ref_surface_m2
            * self.ref_length_m
            * (state.v_air_norm_m_s * c.cn
                + self.ref_length_m / 2.0 * (c.cn_r + c.cn_bd) * state.w_b_rad_s[2]);

        let f = vector![fx, fy, fz];
        let m = vector![mx, my, mz];

        AerodynamicActions {
            forces_b_n: f * state.v_air_b_m_s[0].signum(),
            moments_b_nm: m * state.v_air_b_m_s[0].signum(),
        }
    }
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
