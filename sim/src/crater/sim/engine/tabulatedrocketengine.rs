use super::engine::{RocketEngine, RocketEngineMassProperties};
use crate::math::interp::{find_index, interpolate};

use nalgebra::{Matrix3, Vector3};
use serde_json::Value;

pub struct TabRocketEngine {
    xcg_time: Vec<f64>,
    xcg_value: Vec<f64>,
    thrust_time: Vec<f64>,
    thrust_value: Vec<f64>,
    mass_time: Vec<f64>,
    mass_value: Vec<f64>,
    inertia_xx_time: Vec<f64>,
    inertia_xx_value: Vec<f64>,
    inertia_yy_time: Vec<f64>,
    inertia_yy_value: Vec<f64>,
    inertia_zz_time: Vec<f64>,
    inertia_zz_value: Vec<f64>,
}

impl TabRocketEngine {
    pub fn from_json(json_str: &str) -> Result<Self, serde_json::Error> {
        let data: Value = serde_json::from_str(&std::fs::read_to_string(&json_str).unwrap())?;

        let mut engine = TabRocketEngine {
            xcg_time: Vec::new(),
            xcg_value: Vec::new(),
            thrust_time: Vec::new(),
            thrust_value: Vec::new(),
            mass_time: Vec::new(),
            mass_value: Vec::new(),
            inertia_xx_time: Vec::new(),
            inertia_xx_value: Vec::new(),
            inertia_yy_time: Vec::new(),
            inertia_yy_value: Vec::new(),
            inertia_zz_time: Vec::new(),
            inertia_zz_value: Vec::new(),
        };

        let parse_data = |entries: &Vec<Value>, time: &mut Vec<f64>, value: &mut Vec<f64>| {
            for entry in entries {
                if let Value::Array(pair) = entry {
                    if pair.len() == 2 {
                        if let (Some(t_f64), Some(v_f64)) = (pair[0].as_f64(), pair[1].as_f64()) {
                            time.push(t_f64);
                            value.push(v_f64);
                        }
                    }
                }
            }
        };

        if let Some(thrust) = data["thrust"].as_array() {
            parse_data(thrust, &mut engine.thrust_time, &mut engine.thrust_value);
        }

        if let Some(t_xcg_mass_ixx_iyy_izz) = data["t_xcg_mass_ixx_iyy_izz"].as_array() {
            for entry in t_xcg_mass_ixx_iyy_izz {
                if let Value::Array(values) = entry {
                    if values.len() == 6 {
                        if let (Some(t), Some(xcg), Some(mass), Some(ixx), Some(iyy), Some(izz)) = (
                            values[0].as_f64(),
                            values[1].as_f64(),
                            values[2].as_f64(),
                            values[3].as_f64(),
                            values[4].as_f64(),
                            values[5].as_f64(),
                        ) {
                            engine.xcg_time.push(t);
                            engine.xcg_value.push(xcg);
                            engine.mass_time.push(t);
                            engine.mass_value.push(mass);
                            engine.inertia_xx_time.push(t);
                            engine.inertia_xx_value.push(ixx);
                            engine.inertia_yy_time.push(t);
                            engine.inertia_yy_value.push(iyy);
                            engine.inertia_zz_time.push(t);
                            engine.inertia_zz_value.push(izz);
                        }
                    }
                }
            }
        }

        Ok(engine)
    }
}

impl RocketEngine for TabRocketEngine {
    fn thrust_b(&self, t_sec: f64) -> Vector3<f64> {
        let int = find_index(&self.thrust_time, t_sec);
        Vector3::new(interpolate(&self.thrust_value, int).0, 0.0, 0.0)
    }

    fn mass(&self, t_sec: f64) -> RocketEngineMassProperties {
        let int = find_index(&self.mass_time, t_sec);
        let xcg_int = interpolate(&self.xcg_value, int);
        let mass_int = interpolate(&self.mass_value, int);
        let in_xx_int = interpolate(&self.inertia_xx_value, int);
        let in_yy_int = interpolate(&self.inertia_yy_value, int);
        let in_zz_int = interpolate(&self.inertia_zz_value, int);
        RocketEngineMassProperties {
            xcg_eng_frame_m: xcg_int.0,
            xcg_dot_eng_frame_m: xcg_int.1,
            mass_kg: mass_int.0,
            mass_dot_kg_s: mass_int.1,
            inertia_eng_frame_kgm2: Matrix3::new(
                in_xx_int.0,
                0.0,
                0.0,
                0.0,
                in_yy_int.0,
                0.0,
                0.0,
                0.0,
                in_zz_int.0,
            ),
            inertia_dot_eng_frame_kgm2: Matrix3::new(
                in_xx_int.1,
                0.0,
                0.0,
                0.0,
                in_yy_int.1,
                0.0,
                0.0,
                0.0,
                in_zz_int.1,
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::vec;

    use super::*;

    #[test]
    fn test_from_json() {
        let json_str = r#"{
            "thrust": [[0, 0.0], [0.016, 199.183], [0.024, 260.157]],
            "mass": [[300, 0.0], [200, 1], [100, 2]],
            "xcg": [[0,0.0],[1.0,-1.0]],
            "inertia_xx": [[0, 0.0], [0, 1], [0, 2]],
            "inertia_yy": [[0, 0.0], [0, 1], [0, 2]],
            "inertia_zz": [[0, 0.0], [0, 1], [0, 2]]
        }"#;

        let engine = TabRocketEngine::from_json(json_str).unwrap();

        assert_eq!(engine.thrust_time, vec![0.0, 0.016, 0.024]);
        assert_eq!(engine.thrust_value, vec![0.0, 199.183, 260.157]);

        assert_eq!(engine.xcg_time, vec![0.0, 1.0]);
        assert_eq!(engine.xcg_value, vec![0.0, -1.0]);

        assert_eq!(engine.mass_time, vec![300.0, 200.0, 100.0]);
        assert_eq!(engine.mass_value, vec![0.0, 1.0, 2.0]);

        assert_eq!(engine.inertia_xx_time, vec![0.0, 0.0, 0.0]);
        assert_eq!(engine.inertia_xx_value, vec![0.0, 1.0, 2.0]);

        assert_eq!(engine.inertia_yy_time, vec![0.0, 0.0, 0.0]);
        assert_eq!(engine.inertia_yy_value, vec![0.0, 1.0, 2.0]);

        assert_eq!(engine.inertia_zz_time, vec![0.0, 0.0, 0.0]);
        assert_eq!(engine.inertia_zz_value, vec![0.0, 1.0, 2.0]);
    }
}
