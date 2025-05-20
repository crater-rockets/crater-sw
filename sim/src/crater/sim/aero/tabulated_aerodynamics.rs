use anyhow::{Result, anyhow};
use hdf5_metno::File;
use std::{array, f64, path::Path};
use strum::{AsRefStr, EnumIter, IntoEnumIterator};

use crate::math::interp::Interpolator;

use super::aerodynamics::{
    AeroCoefficientsValues, AeroState, Aerodynamics, AerodynamicsCoefficients,
};

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

pub struct TabulatedAeroCoefficients {
    interp: Interpolator<f32, 8>,
    states: Vec<Vec<f32>>,

    coeffs: Vec<Vec<f32>>,
}

impl AerodynamicsCoefficients for TabulatedAeroCoefficients {
    fn coefficients(&self, state: &AeroState) -> AeroCoefficientsValues {
        self.interpolate(state)
    }
}

impl TabulatedAeroCoefficients {
    pub fn from_h5(file_main: &Path, file_derivatives: &Path) -> Result<Self> {
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
        })
    }

    fn interpolate(&self, state: &AeroState) -> AeroCoefficientsValues {
        let state1 = [
            state.angles.alpha_rad.to_degrees() as f32,
            state.mach as f32,
            state.angles.beta_rad.to_degrees() as f32,
            state.altitude_m as f32,
            state.servo_pos.pos_rad[0].to_degrees() as f32,
            state.servo_pos.pos_rad[1].to_degrees() as f32,
            state.servo_pos.pos_rad[2].to_degrees() as f32,
            state.servo_pos.pos_rad[3].to_degrees() as f32,
        ];

        let state2 = [
            state.angles.beta_tan_rad.to_degrees() as f32,
            state.mach as f32,
            state.angles.alpha_rad.to_degrees() as f32,
            state.altitude_m as f32,
            state.servo_pos.pos_rad[0].to_degrees() as f32,
            state.servo_pos.pos_rad[1].to_degrees() as f32,
            state.servo_pos.pos_rad[2].to_degrees() as f32,
            state.servo_pos.pos_rad[3].to_degrees() as f32,
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

        AeroCoefficientsValues {
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
