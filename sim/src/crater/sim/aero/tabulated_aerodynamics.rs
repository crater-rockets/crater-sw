use anyhow::{Result, anyhow};
use hdf5_metno::File;
use nalgebra::{Vector3, vector};
use std::{array, f64, os::linux::raw::stat, path::Path};
use strum::{AsRefStr, EnumIter, IntoEnumIterator};

use crate::{crater::sim::gnc::ServoPosition, math::interp::Interpolator};

#[derive(Debug, Clone, Copy, AsRefStr, EnumIter)]
enum Coefficients {
    CA,

    CN,
    CNAD,
    CNQ,

    CY,
    CYR,

    CLL,
    CLLP,
    CLLR,

    CM,
    CMAD,
    CMQ,

    CLN,
    CLNP,
    CLNR,
}

#[derive(Debug, Clone, Copy, AsRefStr, EnumIter)]
enum States {
    #[strum(serialize = "alpha")]
    Alpha,
    #[strum(serialize = "mach")]
    Mach,
    #[strum(serialize = "beta")]
    Beta,
    #[strum(serialize = "altitude")]
    Altitude,
    #[strum(serialize = "fin2.delta1")]
    Delta1,
    #[strum(serialize = "fin2.delta2")]
    Delta2,
    #[strum(serialize = "fin2.delta3")]
    Delta3,
    #[strum(serialize = "fin2.delta4")]
    Delta4,
}

pub struct TabulatedAerodynamics {
    interp: Interpolator<f32, 8>,
    states: Vec<Vec<f32>>,

    coeffs: Vec<Vec<f32>>,

    ref_length: f64,
    ref_surface: f64,
}

pub struct AeroState {
    pub servo_pos: ServoPosition,

    pub alpha: f64,
    pub beta: f64,
    pub beta_prime: f64,

    pub mach: f64,
    pub air_density: f64,

    pub v_air_b: Vector3<f64>,
    pub v_air_norm: f64,

    pub w_b: Vector3<f64>,
    pub altitude: f64,
}

impl AeroState {
    pub fn new(
        v_air_b: Vector3<f64>,
        w_b: Vector3<f64>,
        altitude: f64,
        mach: f64,
        air_density: f64,
        servo_pos: ServoPosition,
    ) -> AeroState {
        let v_air_norm = v_air_b.norm();
        AeroState {
            servo_pos,
            alpha: Self::alpha(&v_air_b),
            beta: Self::beta(&v_air_b, v_air_norm),
            beta_prime: Self::beta_prime(&v_air_b),
            mach,
            air_density,
            v_air_b,
            v_air_norm,
            w_b,
            altitude,
        }
    }

    pub fn alpha(v_air_b: &Vector3<f64>) -> f64 {
        if v_air_b[0].abs() >= V_SMALL {
            f64::atan(v_air_b[2] / v_air_b[0])
        } else if v_air_b[2].abs() >= V_SMALL {
            f64::consts::FRAC_PI_2 * v_air_b[2].signum()
        } else {
            0.0
        }
    }

    pub fn beta(v_air_b: &Vector3<f64>, v_air_norm: f64) -> f64 {
        if v_air_norm >= V_SMALL {
            f64::asin(v_air_b[1] / v_air_norm)
        } else if v_air_b[1].abs() >= V_SMALL {
            f64::consts::FRAC_PI_2 * v_air_b[1].signum()
        } else {
            0.0
        }
    }

    pub fn beta_prime(v_air_b: &Vector3<f64>) -> f64 {
        if v_air_b[0].abs() >= V_SMALL {
            f64::atan(v_air_b[1] / v_air_b[0])
        } else if v_air_b[1].abs() >= V_SMALL {
            f64::consts::FRAC_PI_2 * v_air_b[1].signum()
        } else {
            0.0
        }
    }
}

#[allow(nonstandard_style)]
struct InterpolatedCoefficients {
    cA: f64,

    cY: f64,
    cY_r: f64,
    cY_bd: f64,

    cN: f64,
    cN_q: f64,
    cN_ad: f64,

    cl: f64,
    cl_p: f64,
    cl_r: f64,

    cm: f64,
    cm_q: f64,
    cm_ad: f64,

    cn: f64,
    cn_r: f64,
    cn_bd: f64,
}

const V_SMALL: f64 = 1.0e-5;
impl TabulatedAerodynamics {
    pub fn from_h5(
        file_main: &Path,
        file_derivatives: &Path,
        ref_length: f64,
        ref_surface: f64,
    ) -> Result<Self> {
        let h5_main = File::open(file_main)?;
        let h5_derivatives = File::open(file_derivatives)?;

        let mut states = vec![];
        for state in States::iter() {
            let v = h5_main.dataset(state.as_ref())?.read_raw::<f32>()?;
            states.push(v);
        }

        let mut states2 = vec![];
        for state in States::iter() {
            let v = h5_derivatives.dataset(state.as_ref())?.read_raw::<f32>()?;
            states2.push(v);
        }

        if states != states2 {
            return Err(anyhow!(
                "States do not match between two input files: main={:#?}, derivatives={:#?}",
                states,
                states2
            ));
        }

        let mut coeffs = vec![];
        for c in Coefficients::iter() {
            match c {
                Coefficients::CLLP | Coefficients::CLLR => {
                    let v = h5_derivatives.dataset(c.as_ref())?.read_raw::<f32>()?;
                    coeffs.push(v);
                }
                _ => {
                    let v = h5_main.dataset(c.as_ref())?.read_raw::<f32>()?;
                    coeffs.push(v);
                }
            }
        }

        let interp = Interpolator::<f32, 8>::new(array::from_fn(|i| states[i].as_slice()))
            .ok_or_else(|| anyhow!("Bad interpolator"))?;

        Ok(Self {
            interp,
            states,
            coeffs,
            ref_length,
            ref_surface,
        })
    }

    pub fn actions(&self, state: &AeroState) -> (Vector3<f64>, Vector3<f64>) {
        let c = self.interpolate(state);

        let q_v = 0.5 * state.air_density * state.v_air_norm;

        let fx = -q_v * self.ref_surface * (state.v_air_norm * c.cA);
        let fy = q_v
            * self.ref_surface
            * (state.v_air_norm * c.cY + self.ref_length / 2.0 * (c.cY_r + c.cY_bd) * state.w_b[2]);
        let fz = -q_v
            * self.ref_surface
            * (state.v_air_norm * c.cN + self.ref_length / 2.0 * (c.cN_q + c.cN_ad) * state.w_b[2]);

        let mx = q_v
            * self.ref_surface
            * self.ref_length
            * (state.v_air_norm * c.cl
                + self.ref_length / 2.0 * (c.cl_p * state.w_b[0] + c.cl_r * state.w_b[2]));

        let my = q_v
            * self.ref_surface
            * self.ref_length
            * (state.v_air_norm * c.cm + self.ref_length / 2.0 * (c.cm_q + c.cm_ad) * state.w_b[1]);

        let mz = q_v
            * self.ref_surface
            * self.ref_length
            * (state.v_air_norm * c.cn + self.ref_length / 2.0 * (c.cn_r + c.cn_bd) * state.w_b[2]);

        let f = vector![fx, fy, fz];
        let m = vector![mx, my, mz];

        (f * state.v_air_b[0].signum(), m * state.v_air_b[0].signum())
    }

    fn interpolate(&self, state: &AeroState) -> InterpolatedCoefficients {
        let state1 = [
            state.alpha.to_degrees() as f32,
            state.mach as f32,
            state.beta.to_degrees() as f32,
            state.altitude as f32,
            state.servo_pos.0[0] as f32,
            state.servo_pos.0[1] as f32,
            state.servo_pos.0[2] as f32,
            state.servo_pos.0[3] as f32,
        ];

        let state2 = [
            state.beta_prime.to_degrees() as f32,
            state.mach as f32,
            state.alpha.to_degrees() as f32,
            state.altitude as f32,
            state.servo_pos.0[0] as f32,
            state.servo_pos.0[1] as f32,
            state.servo_pos.0[2] as f32,
            state.servo_pos.0[3] as f32,
        ];

        let c1 = [
            self.coeffs[Coefficients::CA as usize].as_slice(),
            self.coeffs[Coefficients::CY as usize].as_slice(),
            self.coeffs[Coefficients::CN as usize].as_slice(),
            self.coeffs[Coefficients::CNQ as usize].as_slice(),
            self.coeffs[Coefficients::CNAD as usize].as_slice(),
            self.coeffs[Coefficients::CLL as usize].as_slice(),
            self.coeffs[Coefficients::CLLP as usize].as_slice(),
            self.coeffs[Coefficients::CLLR as usize].as_slice(),
            self.coeffs[Coefficients::CM as usize].as_slice(),
            self.coeffs[Coefficients::CMQ as usize].as_slice(),
            self.coeffs[Coefficients::CMAD as usize].as_slice(),
            self.coeffs[Coefficients::CLN as usize].as_slice(),
        ];

        let mut v1: [f32; 12] = [0f32; 12];

        let c2 = [
            self.coeffs[Coefficients::CNQ as usize].as_slice(), // CYR
            self.coeffs[Coefficients::CNAD as usize].as_slice(), // CYBD
            self.coeffs[Coefficients::CMQ as usize].as_slice(), // CLNR
            self.coeffs[Coefficients::CMAD as usize].as_slice(), // CLNBD
        ];
        let mut v2: [f32; 4] = [0f32; 4];

        self.interp.interpn(&state1, &c1, &mut v1);
        self.interp.interpn(&state2, &c2, &mut v2);

        InterpolatedCoefficients {
            cA: v1[0] as f64,

            cY: v1[1] as f64,
            cY_r: v2[0] as f64,
            cY_bd: v2[1] as f64,

            cN: v1[2] as f64,
            cN_q: v1[3] as f64,
            cN_ad: v1[4] as f64,

            cl: v1[5] as f64,
            cl_p: v1[6] as f64,
            cl_r: v1[7] as f64,

            cm: v1[8] as f64,
            cm_q: v1[9] as f64,
            cm_ad: v1[10] as f64,

            cn: v1[11] as f64,
            cn_r: v2[2] as f64,
            cn_bd: v2[3] as f64,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{f64, path::PathBuf, str::FromStr};

    use nalgebra::vector;

    use crate::crater::sim::gnc::ServoPosition;

    use super::{AeroState, TabulatedAerodynamics};

    #[test]
    fn abcd() {
        let file =
            PathBuf::from_str("/home/luca/code/crater/pydatcom/for006_coeffs_97.h5").unwrap();

        let aero =
            TabulatedAerodynamics::from_h5(&file, &file, 0.15, f64::consts::PI * 0.15f64.powf(2.0))
                .unwrap();

        let state = AeroState::new(
            vector![100.0, 0.0, 8.74886635],
            vector![0.0, 0.0, 0.0],
            1400.0,
            0.2,
            1.0,
            ServoPosition(vector![0.0, 0.0, 0.0, 0.0]),
        );

        let (f, m) = aero.actions(&state);

        let state = AeroState::new(
            vector![100.0, 8.74886635, 0.0],
            vector![0.0, 0.0, 0.0],
            1400.0,
            0.2,
            1.0,
            ServoPosition(vector![0.0, 0.0, 0.0, 0.0]),
        );

        let (f, m) = aero.actions(&state);

        println!("f: {:#?}, m: {:#?}", f, m);
    }
}
