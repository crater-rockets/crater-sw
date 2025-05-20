use super::aerodynamics::{AeroState, AerodynamicActions, Aerodynamics};
use crate::parameters::ParameterMap;
use anyhow::Result;
use core::f64;
use nalgebra::vector;
use num_traits::Pow;

#[allow(nonstandard_style)]
pub struct LinAeroCoefficients {
    cA_0: f64,
    cA_a: f64,
    cA_b: f64,
    cA_dy: f64,
    cA_dp: f64,
    cA_dr: f64,
    cA_ds: f64,

    cY_b: f64,
    cY_r: f64,
    cY_dy: f64,

    cN_a: f64,
    cN_q: f64,
    cN_dp: f64,

    cl_0: f64,
    cl_p: f64,
    cl_dr: f64,

    cm_a: f64,
    cm_q: f64,
    cm_dp: f64,

    cn_b: f64,
    cn_r: f64,
    cn_dy: f64,
}

impl LinAeroCoefficients {
    pub fn from_params(params: &ParameterMap) -> Result<Self> {
        Ok(Self {
            cA_0: params.get_param("cA_0")?.value_float()?,
            cA_a: params.get_param("cA_a")?.value_float()?,
            cA_b: params.get_param("cA_b")?.value_float()?,
            cA_dy: params.get_param("cA_dy")?.value_float()?,
            cA_dp: params.get_param("cA_dp")?.value_float()?,
            cA_dr: params.get_param("cA_dr")?.value_float()?,
            cA_ds: params.get_param("cA_ds")?.value_float()?,

            cY_b: params.get_param("cY_b")?.value_float()?,
            cY_r: params.get_param("cY_r")?.value_float()?,
            cY_dy: params.get_param("cY_dy")?.value_float()?,

            cN_a: params.get_param("cN_a")?.value_float()?,
            cN_q: params.get_param("cN_q")?.value_float()?,
            cN_dp: params.get_param("cN_dp")?.value_float()?,

            cl_0: params.get_param("cl_0")?.value_float()?,
            cl_p: params.get_param("cl_p")?.value_float()?,
            cl_dr: params.get_param("cl_dr")?.value_float()?,

            cm_a: params.get_param("cm_a")?.value_float()?,
            cm_q: params.get_param("cm_q")?.value_float()?,
            cm_dp: params.get_param("cm_dp")?.value_float()?,

            cn_b: params.get_param("cn_b")?.value_float()?,
            cn_r: params.get_param("cn_r")?.value_float()?,
            cn_dy: params.get_param("cn_dy")?.value_float()?,
        })
    }
}

pub struct LinearAerodynamics {
    diameter_m: f64,
    surface_m2: f64,
    c: LinAeroCoefficients,
}

impl LinearAerodynamics {
    pub fn new(diameter: f64, surface: f64, coefficients: LinAeroCoefficients) -> Self {
        LinearAerodynamics {
            diameter_m: diameter,
            surface_m2: surface,
            c: coefficients,
        }
    }
}

impl Aerodynamics for LinearAerodynamics {
    #[allow(non_snake_case)]
    fn actions(&self, state: &AeroState) -> AerodynamicActions {
        let alpha = state.angles.alpha_rad;
        let beta = state.angles.beta_rad;
        let mixed_servo_pos = state.servo_pos.mix();

        let cA = self.c.cA_0
            + (self.c.cA_a * alpha.pow(2.0) + self.c.cA_b * beta.pow(2.0))
            + (self.c.cA_dy * mixed_servo_pos.yaw().powf(2.0))
            + (self.c.cA_dp * mixed_servo_pos.pitch().powf(2.0))
            + (self.c.cA_dr * mixed_servo_pos.roll().powf(2.0))
            + (self.c.cA_ds * mixed_servo_pos.squeeze().powf(2.0));

        let cY = self.c.cY_b * beta + self.c.cY_dy * mixed_servo_pos.yaw();
        let cN = self.c.cN_a * alpha + self.c.cN_dp * mixed_servo_pos.pitch();

        let cl = self.c.cl_0 + self.c.cl_dr * mixed_servo_pos.roll();
        let cm = self.c.cm_a * alpha + self.c.cm_dp * mixed_servo_pos.pitch();
        let cn = self.c.cn_b * beta + self.c.cn_dy * mixed_servo_pos.yaw();

        let q_v = 0.5 * state.air_density_kg_m3 * state.v_air_norm_m_s;
        let q = q_v * state.v_air_norm_m_s;

        let f = vector![
            -q * self.surface_m2 * cA,
            q * self.surface_m2 * cY
                + q_v * self.surface_m2 * self.diameter_m * self.c.cY_r * state.w_b_rad_s[2],
            -q * self.surface_m2 * cN
                - q_v * self.surface_m2 * self.diameter_m * self.c.cN_q * state.w_b_rad_s[1],
        ];

        let t = vector![
            q * self.surface_m2 * cl * self.diameter_m
                + 0.5
                    * q_v
                    * self.surface_m2
                    * self.diameter_m.pow(2.0)
                    * self.c.cl_p
                    * state.w_b_rad_s[0],
            q * self.surface_m2 * cm * self.diameter_m
                + 0.5
                    * q_v
                    * self.surface_m2
                    * self.diameter_m.pow(2.0)
                    * self.c.cm_q
                    * state.w_b_rad_s[1],
            q * self.surface_m2 * cn * self.diameter_m
                + 0.5
                    * q_v
                    * self.surface_m2
                    * self.diameter_m.pow(2.0)
                    * self.c.cn_r
                    * state.w_b_rad_s[2],
        ];

        AerodynamicActions {
            forces_b_n: f * state.v_air_b_m_s[0].signum(),
            moments_b_nm: t * state.v_air_b_m_s[0].signum(),
        }
    }
}
