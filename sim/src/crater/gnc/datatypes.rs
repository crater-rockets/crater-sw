use nalgebra::{Matrix4, Vector4, matrix};

/// From fin deflections to mixed deflections
const MIXING_MATRIX: Matrix4<f64> = matrix![-0.25,  0.25,  0.25, -0.25;
                                             0.25,  0.25, -0.25, -0.25;
                                            -0.25, -0.25, -0.25, -0.25;
                                            -0.25,  0.25, -0.25,  0.25];

/// From mixed deflections to fin deflections
const INV_MIXING_MATRIX: Matrix4<f64> = matrix![-1.0,  1.0, -1.0, -1.0;
                                                 1.0,  1.0, -1.0,  1.0;
                                                 1.0, -1.0, -1.0, -1.0;
                                                -1.0, -1.0, -1.0,  1.0];

/// Fin numbering (view from back)
/// ```txt
/// (2)       (3)
///    \     /
///     \___/
///     |___|------> body Y axis
///     / | \
///    /  |  \
/// (1)   |   (4)
///       v
///  body Z axis
/// ```
/// Positive angle according to right hand rule over fin hinge axis
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ServoPosition {
    pub pos_rad: Vector4<f64>,
}

impl ServoPosition {
    pub fn mix(&self) -> MixedServoPosition {
        MixedServoPosition {
            pos_rad: MIXING_MATRIX * self.pos_rad,
        }
    }
}

impl From<Vector4<f64>> for ServoPosition {
    fn from(pos_rad: Vector4<f64>) -> Self {
        ServoPosition { pos_rad }
    }
}

impl From<[f64; 4]> for ServoPosition {
    fn from(pos_rad: [f64; 4]) -> Self {
        ServoPosition {
            pos_rad: pos_rad.into(),
        }
    }
}

/// Fin mixing
/// ```txt
///      Yaw                   Pitch                   Roll                 Squeeze          
/// (2)       (3)          (2)       (3)           (2)       (3)         (2)       (3)     
///   <\    </               <\     />                \>    />             <\     />       
///     \___/                  \___/                   \___/                 \___/         
///     |___|------> Y         |___|------> Y          |___|------> Y        |___|------> Y
///     / | \                  / | \                   / | \                 / | \         
///   </  | <\                /> | <\                </  | <\              </  |  \>
/// (1)   |   (4)          (1)   |   (4)           (1)   |   (4)         (1)   |   (4)     
///       v                      v                       v                     v           
///       Z                      Z                       Z                     Z           
///
/// δ_yaw      = (- δ_1 + δ_2 + δ_3 - δ_4) / 4
/// δ_pitch    = (+ δ_1 + δ_2 - δ_3 - δ_4) / 4
/// δ_roll     = (- δ_1 - δ_2 - δ_3 - δ_4) / 4
/// δ_squeeze  = (- δ_1 + δ_2 - δ_3 + δ_4) / 4
///
/// |δ_y|   | -1  1  1 -1 | |δ_1|
/// |δ_p| = |  1  1 -1 -1 | |δ_2| * 1/4
/// |δ_r|   | -1 -1 -1 -1 | |δ_3|
/// |δ_s|   | -1  1 -1  1 | |δ_4|
///
/// ```
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MixedServoPosition {
    pub pos_rad: Vector4<f64>,
}

impl MixedServoPosition {
    pub fn unmix(&self) -> ServoPosition {
        ServoPosition {
            pos_rad: INV_MIXING_MATRIX * self.pos_rad,
        }
    }

    pub fn yaw_rad(&self) -> f64 {
        self.pos_rad[0]
    }

    pub fn pitch_rad(&self) -> f64 {
        self.pos_rad[1]
    }

    pub fn roll_rad(&self) -> f64 {
        self.pos_rad[2]
    }

    pub fn squeeze_rad(&self) -> f64 {
        self.pos_rad[3]
    }
}

impl From<Vector4<f64>> for MixedServoPosition {
    fn from(mixed_rad: Vector4<f64>) -> Self {
        MixedServoPosition { pos_rad: mixed_rad }
    }
}

impl From<[f64; 4]> for MixedServoPosition {
    fn from(mixed_rad: [f64; 4]) -> Self {
        MixedServoPosition {
            pos_rad: mixed_rad.into(),
        }
    }
}

#[cfg(test)]
mod test {
    use approx::assert_relative_eq;

    use super::*;
    use nalgebra::vector;

    #[test]
    fn test_inverse_matrix() {
        let test_positions = [
            ServoPosition(vector![1.0, 2.0, 3.0, 4.0]),
            ServoPosition(vector![0.0, 0.0, 0.0, 0.0]),
            ServoPosition(vector![-1.0, 2.0, -3.0, 4.0]),
            ServoPosition(vector![1.0, 2.0, -3.0, -4.0]),
        ];

        for p in test_positions {
            let mixed = p.mix();

            let unmixed = mixed.unmix();

            assert_eq!(p, unmixed)
        }
    }

    #[test]
    fn test_mixing() {
        let test_positions = [
            (
                ServoPosition(vector![-1.0, 1.0, 1.0, -1.0]),
                MixedServoPosition(vector![1.0, 0.0, 0.0, 0.0]),
            ),
            (
                ServoPosition(vector![1.0, 1.0, -1.0, -1.0]),
                MixedServoPosition(vector![0.0, 1.0, 0.0, 0.0]),
            ),
            (
                ServoPosition(vector![-1.0, -1.0, -1.0, -1.0]),
                MixedServoPosition(vector![0.0, 0.0, 1.0, 0.0]),
            ),
            (
                ServoPosition(vector![-1.0, 1.0, -1.0, 1.0]),
                MixedServoPosition(vector![0.0, 0.0, 0.0, 1.0]),
            ),
        ];

        for (p, expected) in test_positions.iter() {
            let mixed = p.mix();

            assert_relative_eq!(mixed.0, expected.0, epsilon = 0.001);
        }

        for (expected, mixed) in test_positions.iter() {
            let pos = mixed.unmix();

            assert_relative_eq!(pos.0, expected.0, epsilon = 0.001);
        }
    }
}
