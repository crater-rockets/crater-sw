use core::f64;

use nalgebra::{vector, Vector3};
use num_traits::Pow;

#[allow(nonstandard_style)]
pub struct Coefficients {
    cA_0: f64,
    cA_a: f64,
    cA_b: f64,
    cY_b: f64,
    cY_r: f64,
    cN_a: f64,
    cN_q: f64,
    cl_0: f64,
    cl_p: f64,
    cm_a: f64,
    cm_q: f64,
    cn_b: f64,
    cn_r: f64,
}

impl Default for Coefficients {
    fn default() -> Self {
        Coefficients {
            cA_0: 0.3200,
            cA_a: -0.1725,
            cA_b: -0.1725,
            cY_b: -24.0744,
            cY_r: 154.0,
            cN_a: 24.0744,
            cN_q: 154.0,
            cl_0: 0.0,
            cl_p: -19.0,
            cm_a: -37.2959,
            cm_q: -1813.0,
            cn_b: 37.2959,
            cn_r: -1813.0,
        }
    }
}

const V_SMALL: f64 = 1.0e-8;
pub struct Aerodynamics {
    v_air_b: Vector3<f64>,
    v_norm: f64,
    w: Vector3<f64>,
    rho: f64,
    diameter: f64,
    surface: f64,
}

impl Aerodynamics {
    pub fn new(
        v_b: Vector3<f64>,
        v_wind_b: Vector3<f64>,
        w: Vector3<f64>,
        diameter: f64,
        surface: f64,
    ) -> Self {
        let v_air_b = v_b - v_wind_b;
        let v_norm = v_air_b.norm();

        Aerodynamics {
            v_air_b,
            v_norm,
            w,
            rho: 1.0,
            diameter,
            surface,
        }
    }

    pub fn alpha(&self) -> f64 {
        if self.v_air_b[0].abs() >= V_SMALL {
            f64::atan(self.v_air_b[2] / self.v_air_b[0])
        } else if self.v_air_b[2].abs() >= V_SMALL {
            f64::consts::FRAC_PI_2 * self.v_air_b[2].signum()
        } else {
            0.0
        }
    }

    pub fn beta(&self) -> f64 {
        if self.v_norm >= V_SMALL {
            f64::asin(self.v_air_b[1] / self.v_norm)
        } else if self.v_air_b[1].abs() >= V_SMALL {
            f64::consts::FRAC_PI_2 * self.v_air_b[1].signum()
        } else {
            0.0
        }
    }

    pub fn actions(&self, c: &Coefficients) -> (Vector3<f64>, Vector3<f64>) {
        let alpha = self.alpha();
        let beta = self.beta();

        let cA =
            (c.cA_0 + c.cA_a * alpha.pow(2.0) + c.cA_b * beta.pow(2.0)) * self.v_air_b[0].signum();
        let cY = (c.cY_b * beta) * self.v_air_b[0].signum();
        let cN = (c.cN_a * alpha) * self.v_air_b[0].signum();

        let cl = c.cl_0 * self.v_air_b[0].signum();
        let cm = c.cm_a * alpha * self.v_air_b[0].signum();
        let cn = c.cn_b * beta * self.v_air_b[0].signum();

        let q_v = 0.5 * self.rho * self.v_norm;
        let q = q_v * self.v_norm;

        let f = vector![
            -q * self.surface * cA,
            q * self.surface * cY + q_v * self.surface * self.diameter * c.cY_r * self.v_air_b[2],
            -q * self.surface * cN - q_v * self.surface * self.diameter * c.cN_q * self.v_air_b[1],
        ];

        let t = vector![
            q * self.surface * cl * self.diameter
                + 0.5 * q_v * self.surface * self.diameter.pow(2.0) * c.cl_p * self.v_air_b[0],
            q * self.surface * cm * self.diameter
                + 0.5 * q_v * self.surface * self.diameter.pow(2.0) * c.cm_q * self.v_air_b[1],
            q * self.surface * cn * self.diameter
                + 0.5 * q_v * self.surface * self.diameter.pow(2.0) * c.cn_r * self.v_air_b[2],
        ];

        (f, t)
    }
}
