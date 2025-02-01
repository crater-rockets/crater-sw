use anyhow::Result;
use core::f64;
use nalgebra::{vector, Vector3};
use num_traits::Pow;

use crate::{
    crater::sim::gnc::{MixedServoPosition, ServoPosition},
    parameters::ParameterService,
};

use super::atmosphere::Atmosphere;

#[allow(nonstandard_style)]
pub struct AeroCoefficients {
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

impl AeroCoefficients {
    pub fn from_params(basepath: &str, params: &ParameterService) -> Result<Self> {
        Ok(Self {
            cA_0: params.get_f64(format!("{basepath}/aero/cA_0").as_str())?,
            cA_a: params.get_f64(format!("{basepath}/aero/cA_a").as_str())?,
            cA_b: params.get_f64(format!("{basepath}/aero/cA_b").as_str())?,
            cA_dy: params.get_f64(format!("{basepath}/aero/cA_dy").as_str())?,
            cA_dp: params.get_f64(format!("{basepath}/aero/cA_dp").as_str())?,
            cA_dr: params.get_f64(format!("{basepath}/aero/cA_dr").as_str())?,
            cA_ds: params.get_f64(format!("{basepath}/aero/cA_ds").as_str())?,

            cY_b: params.get_f64(format!("{basepath}/aero/cY_b").as_str())?,
            cY_r: params.get_f64(format!("{basepath}/aero/cY_r").as_str())?,
            cY_dy: params.get_f64(format!("{basepath}/aero/cY_dy").as_str())?,

            cN_a: params.get_f64(format!("{basepath}/aero/cN_a").as_str())?,
            cN_q: params.get_f64(format!("{basepath}/aero/cN_q").as_str())?,
            cN_dp: params.get_f64(format!("{basepath}/aero/cN_dp").as_str())?,

            cl_0: params.get_f64(format!("{basepath}/aero/cl_0").as_str())?,
            cl_p: params.get_f64(format!("{basepath}/aero/cl_p").as_str())?,
            cl_dr: params.get_f64(format!("{basepath}/aero/cl_dr").as_str())?,

            cm_a: params.get_f64(format!("{basepath}/aero/cm_a").as_str())?,
            cm_q: params.get_f64(format!("{basepath}/aero/cm_q").as_str())?,
            cm_dp: params.get_f64(format!("{basepath}/aero/cm_dp").as_str())?,

            cn_b: params.get_f64(format!("{basepath}/aero/cn_b").as_str())?,
            cn_r: params.get_f64(format!("{basepath}/aero/cn_r").as_str())?,
            cn_dy: params.get_f64(format!("{basepath}/aero/cn_dy").as_str())?,
        })
    }
}

const V_SMALL: f64 = 1.0e-5;

pub struct AeroState {
    pub mixed_servo_pos: MixedServoPosition,
    pub v_air_b: Vector3<f64>,
    pub v_norm: f64,
    pub w: Vector3<f64>,
    pub alt: f64,
}

impl AeroState {
    pub fn new(
        servo_pos: MixedServoPosition,
        v_b: Vector3<f64>,
        v_wind_b: Vector3<f64>,
        w: Vector3<f64>,
        alt: f64,
    ) -> AeroState {
        let v_air_b = v_b - v_wind_b;
        let v_norm = v_air_b.norm();

        AeroState {
            mixed_servo_pos: servo_pos,
            v_air_b,
            v_norm,
            w,
            alt,
        }
    }
}

pub struct AerodynamicsResult {
    pub alpha: f64,
    pub beta: f64,
    pub forces: Vector3<f64>,
    pub moments: Vector3<f64>,
}

pub struct Aerodynamics {
    atmosphere: Box<dyn Atmosphere + Send>,
    diameter: f64,
    surface: f64,
    c: AeroCoefficients,
}

impl Aerodynamics {
    pub fn new(
        diameter: f64,
        surface: f64,
        atmosphere: Box<dyn Atmosphere + Send>,
        coefficients: AeroCoefficients,
    ) -> Self {
        Aerodynamics {
            atmosphere,
            diameter,
            surface,
            c: coefficients,
        }
    }

    pub fn calc(&self, state: &AeroState) -> AerodynamicsResult {
        let alpha = self.alpha(state);
        let beta = self.beta(state);

        let (forces, moments) = self.actions(alpha, beta, state);

        AerodynamicsResult {
            alpha,
            beta,
            forces,
            moments,
        }
    }

    pub fn alpha(&self, state: &AeroState) -> f64 {
        if state.v_air_b[0].abs() >= V_SMALL {
            f64::atan(state.v_air_b[2] / state.v_air_b[0])
        } else if state.v_air_b[2].abs() >= V_SMALL {
            f64::consts::FRAC_PI_2 * state.v_air_b[2].signum()
        } else {
            0.0
        }
    }

    pub fn beta(&self, state: &AeroState) -> f64 {
        if state.v_norm >= V_SMALL {
            f64::asin(state.v_air_b[1] / state.v_norm)
        } else if state.v_air_b[1].abs() >= V_SMALL {
            f64::consts::FRAC_PI_2 * state.v_air_b[1].signum()
        } else {
            0.0
        }
    }

    #[allow(non_snake_case)]
    pub fn actions(
        &self,
        alpha: f64,
        beta: f64,
        state: &AeroState,
    ) -> (Vector3<f64>, Vector3<f64>) {
        let cA = self.c.cA_0
            + (self.c.cA_a * alpha.pow(2.0) + self.c.cA_b * beta.pow(2.0))
            + (self.c.cA_dy * state.mixed_servo_pos.yaw().powf(2.0))
            + (self.c.cA_dp * state.mixed_servo_pos.pitch().powf(2.0))
            + (self.c.cA_dr * state.mixed_servo_pos.roll().powf(2.0))
            + (self.c.cA_ds * state.mixed_servo_pos.squeeze().powf(2.0));

        let cY = self.c.cY_b * beta + self.c.cY_dy * state.mixed_servo_pos.yaw();
        let cN = self.c.cN_a * alpha + self.c.cN_dp * state.mixed_servo_pos.pitch();

        let cl = self.c.cl_0 + self.c.cl_dr * state.mixed_servo_pos.roll();
        let cm = self.c.cm_a * alpha + self.c.cm_dp * state.mixed_servo_pos.pitch();
        let cn = self.c.cn_b * beta + self.c.cn_dy * state.mixed_servo_pos.yaw();

        let q_v = 0.5 * self.atmosphere.density(state.alt) * state.v_norm;
        let q = q_v * state.v_norm;

        let f = vector![
            -q * self.surface * cA,
            q * self.surface * cY + q_v * self.surface * self.diameter * self.c.cY_r * state.w[2],
            -q * self.surface * cN - q_v * self.surface * self.diameter * self.c.cN_q * state.w[1],
        ];

        let t = vector![
            q * self.surface * cl * self.diameter
                + 0.5 * q_v * self.surface * self.diameter.pow(2.0) * self.c.cl_p * state.w[0],
            q * self.surface * cm * self.diameter
                + 0.5 * q_v * self.surface * self.diameter.pow(2.0) * self.c.cm_q * state.w[1],
            q * self.surface * cn * self.diameter
                + 0.5 * q_v * self.surface * self.diameter.pow(2.0) * self.c.cn_r * state.w[2],
        ];

        (f * state.v_air_b[0].signum(), t * state.v_air_b[0].signum())
    }
}
