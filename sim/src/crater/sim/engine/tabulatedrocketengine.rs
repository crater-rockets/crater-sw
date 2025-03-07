use super::engine::{RocketEngine, RocketEngineMasses};
use crate::math::interp::{self, interp, interp_slope};
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

        let parse_data = |data: &Value, time: &mut Vec<f64>, value: &mut Vec<f64>| {
            if let Value::Array(entries) = data {
                for entry in entries {
                    if let Value::Array(pair) = entry {
                        if pair.len() == 2 {
                            if let (Value::Number(t), Value::Number(v)) = (&pair[0], &pair[1]) {
                                if let (Some(t_f64), Some(v_f64)) = (t.as_f64(), v.as_f64()) {
                                    time.push(t_f64);
                                    value.push(v_f64);
                                }
                            }
                        }
                    }
                }
            }
        };

        if let Value::Array(xcg) = &data["xcg"] {
            parse_data(
                &Value::Array(xcg.clone()),
                &mut engine.xcg_time,
                &mut engine.xcg_value,
            );
        }
        if let Value::Array(thrust) = &data["thrust"] {
            parse_data(
                &Value::Array(thrust.clone()),
                &mut engine.thrust_time,
                &mut engine.thrust_value,
            );
        }
        if let Value::Array(mass) = &data["mass"] {
            parse_data(
                &Value::Array(mass.clone()),
                &mut engine.mass_time,
                &mut engine.mass_value,
            );
        }
        if let Value::Array(inertia_xx) = &data["inertia_xx"] {
            parse_data(
                &Value::Array(inertia_xx.clone()),
                &mut engine.inertia_xx_time,
                &mut engine.inertia_xx_value,
            );
        }
        if let Value::Array(inertia_yy) = &data["inertia_yy"] {
            parse_data(
                &Value::Array(inertia_yy.clone()),
                &mut engine.inertia_yy_time,
                &mut engine.inertia_yy_value,
            );
        }
        if let Value::Array(inertia_zz) = &data["inertia_zz"] {
            parse_data(
                &Value::Array(inertia_zz.clone()),
                &mut engine.inertia_zz_time,
                &mut engine.inertia_zz_value,
            );
        }

        Ok(engine)
    }
}

impl RocketEngine for TabRocketEngine {
    fn thrust_b(&self, t: f64) -> Vector3<f64> {
        Vector3::new(
            interp(
                &self.thrust_time,
                &self.thrust_value,
                t,
                &interp::InterpMode::Constant(0.0),
            ),
            0.0,
            0.0,
        )
    }

    fn masses_prop(&self, t: f64) -> RocketEngineMasses {
        RocketEngineMasses {
            xcg: interp(
                &self.xcg_time,
                &self.mass_value,
                t,
                &interp::InterpMode::FirstLast,
            ),
            xcg_dot: interp_slope(
                &self.xcg_time,
                &self.mass_value,
                t,
                &interp::InterpMode::Constant(0.0),
            ),
            mass: interp(
                &self.mass_time,
                &self.mass_value,
                t,
                &interp::InterpMode::FirstLast,
            ),
            mass_dot: interp_slope(
                &self.mass_time,
                &self.mass_value,
                t,
                &interp::InterpMode::Constant(0.0),
            ),
            inertia: Matrix3::new(
                interp(
                    &self.inertia_xx_time,
                    &self.inertia_xx_value,
                    t,
                    &interp::InterpMode::FirstLast,
                ),
                0.0,
                0.0,
                0.0,
                interp(
                    &self.inertia_yy_time,
                    &self.inertia_yy_value,
                    t,
                    &interp::InterpMode::FirstLast,
                ),
                0.0,
                0.0,
                0.0,
                interp(
                    &self.inertia_zz_time,
                    &self.inertia_zz_value,
                    t,
                    &interp::InterpMode::FirstLast,
                ),
            ),
            inertia_dot: Matrix3::new(
                interp_slope(
                    &self.inertia_xx_time,
                    &self.inertia_xx_value,
                    t,
                    &interp::InterpMode::Constant(0.0),
                ),
                0.0,
                0.0,
                0.0,
                interp_slope(
                    &self.inertia_yy_time,
                    &self.inertia_yy_value,
                    t,
                    &interp::InterpMode::Constant(0.0),
                ),
                0.0,
                0.0,
                0.0,
                interp_slope(
                    &self.inertia_zz_time,
                    &self.inertia_zz_value,
                    t,
                    &interp::InterpMode::Constant(0.0),
                ),
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
