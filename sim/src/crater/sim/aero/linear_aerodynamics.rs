use super::aerodynamics::{AeroCoefficientsValues, AeroState, AerodynamicsCoefficients};
use crate::parameters::ParameterMap;
use anyhow::Result;
use core::f64;
use num_traits::Pow;

#[allow(nonstandard_style)]
pub struct LinearizedAeroCoefficients {
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

impl LinearizedAeroCoefficients {
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

impl AerodynamicsCoefficients for LinearizedAeroCoefficients {
    #[allow(non_snake_case)]
    fn coefficients(&self, state: &AeroState) -> super::aerodynamics::AeroCoefficientsValues {
        let alpha = state.angles.alpha_rad;
        let beta = state.angles.beta_rad;
        let mixed_servo_pos = state.servo_pos.mix();

        let cA = self.cA_0
            + (self.cA_a * alpha.pow(2.0) + self.cA_b * beta.pow(2.0))
            + (self.cA_dy * mixed_servo_pos.yaw_rad().powf(2.0))
            + (self.cA_dp * mixed_servo_pos.pitch_rad().powf(2.0))
            + (self.cA_dr * mixed_servo_pos.roll_rad().powf(2.0))
            + (self.cA_ds * mixed_servo_pos.squeeze_rad().powf(2.0));

        let cY = self.cY_b * beta + self.cY_dy * mixed_servo_pos.yaw_rad();
        let cN = self.cN_a * alpha + self.cN_dp * mixed_servo_pos.pitch_rad();

        let cl = self.cl_0 + self.cl_dr * mixed_servo_pos.roll_rad();
        let cm = self.cm_a * alpha + self.cm_dp * mixed_servo_pos.pitch_rad();
        let cn = self.cn_b * beta + self.cn_dy * mixed_servo_pos.yaw_rad();

        AeroCoefficientsValues {
            cA,
            cY,
            cY_r: self.cY_r,
            cY_bd: 0.0,

            cN,
            cN_q: self.cN_q,
            cN_ad: 0.0,

            cl,
            cl_p: self.cl_p,
            cl_r: 0.0,

            cm,

            cm_q: self.cm_q,
            cm_ad: 0.0,

            cn,
            cn_r: self.cn_r,
            cn_bd: 0.0,
        }
    }
}
