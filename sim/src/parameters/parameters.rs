use std::collections::{btree_map, BTreeMap};

use serde::{Deserialize, Serialize};
use thiserror::Error;
use toml::{Table, Value};

#[derive(Debug, Clone, Error, PartialEq, Eq)]
pub enum Error {
    #[error("Error deserializing parameters")]
    Deserialize(#[from] toml::de::Error),

    #[error("Parameter toml does not have the right structure (error in '{0}')")]
    BadToml(String),

    #[error("Element '{path}' not found")]
    NotFound { path: String },

    #[error("Cannot cast parameter '{path}' to {dtype}")]
    BadCast { path: String, dtype: String },

    #[error("Element '{path}' is not a parameter")]
    NotAParameter { path: String },

    #[error("Element '{path}' is not a map")]
    NotAMap { path: String },
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "type")]
pub enum FloatDistribution {
    #[serde(rename = "normal")]
    Normal { mean: f64, variance: f64 },
    #[serde(rename = "uniform")]
    Uniform { min: f64, max: f64 },
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct RandFloat {
    val: f64,
    sampled: Option<f64>,
    dist: FloatDistribution,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "type")]
pub enum ParameterValue {
    #[serde(rename = "bool")]
    Bool { val: bool },
    #[serde(rename = "int")]
    Int { val: i64 },
    #[serde(rename = "float")]
    Float { val: f64 },
    #[serde(rename = "str")]
    String { val: String },
    #[serde(rename = "randfloat")]
    RandFloat(RandFloat),

    #[serde(rename = "bool[]")]
    BoolArray { val: Vec<bool> },
    #[serde(rename = "int[]")]
    IntArray { val: Vec<i64> },
    #[serde(rename = "float[]")]
    FloatArray { val: Vec<f64> },
    #[serde(rename = "str[]")]
    StringArray { val: Vec<String> },
    #[serde(rename = "randfloat[]")]
    RandFloatArray { val: Vec<RandFloat> },
}

#[derive(Debug, Clone, PartialEq)]
pub struct Parameter {
    path: String,
    value: ParameterValue,
}

impl Parameter {
    pub fn path(&self) -> &str {
        &self.path
    }

    pub fn value_bool(&self) -> Result<bool, Error> {
        if let ParameterValue::Bool { val } = self.value {
            Ok(val)
        } else {
            Err(Error::BadCast {
                path: self.path.clone(),
                dtype: "bool".to_string(),
            })
        }
    }

    pub fn value_int(&self) -> Result<i64, Error> {
        if let ParameterValue::Int { val } = self.value {
            Ok(val)
        } else {
            Err(Error::BadCast {
                path: self.path.clone(),
                dtype: "int".to_string(),
            })
        }
    }

    pub fn value_float(&self) -> Result<f64, Error> {
        if let ParameterValue::Float { val } = self.value {
            Ok(val)
        } else {
            Err(Error::BadCast {
                path: self.path.clone(),
                dtype: "float".to_string(),
            })
        }
    }

    pub fn value_string(&self) -> Result<String, Error> {
        if let ParameterValue::String { val } = &self.value {
            Ok(val.clone())
        } else {
            Err(Error::BadCast {
                path: self.path.clone(),
                dtype: "str".to_string(),
            })
        }
    }

    pub fn value_randfloat(&self) -> Result<RandFloat, Error> {
        if let ParameterValue::RandFloat(val) = &self.value {
            Ok(val.clone())
        } else {
            Err(Error::BadCast {
                path: self.path.clone(),
                dtype: "randfloat".to_string(),
            })
        }
    }

    pub fn value_bool_arr(&self) -> Result<&[bool], Error> {
        if let ParameterValue::BoolArray { val } = &self.value {
            Ok(val)
        } else {
            Err(Error::BadCast {
                path: self.path.clone(),
                dtype: "bool[]".to_string(),
            })
        }
    }

    pub fn value_int_arr(&self) -> Result<&[i64], Error> {
        if let ParameterValue::IntArray { val } = &self.value {
            Ok(val)
        } else {
            Err(Error::BadCast {
                path: self.path.clone(),
                dtype: "int[]".to_string(),
            })
        }
    }

    pub fn value_float_arr(&self) -> Result<&[f64], Error> {
        if let ParameterValue::FloatArray { val } = &self.value {
            Ok(val)
        } else {
            Err(Error::BadCast {
                path: self.path.clone(),
                dtype: "float[]".to_string(),
            })
        }
    }

    pub fn value_string_arr(&self) -> Result<&[String], Error> {
        if let ParameterValue::StringArray { val } = &self.value {
            Ok(val)
        } else {
            Err(Error::BadCast {
                path: self.path.clone(),
                dtype: "str[]".to_string(),
            })
        }
    }

    pub fn value_randfloat_arr(&self) -> Result<&[RandFloat], Error> {
        if let ParameterValue::RandFloatArray { val } = &self.value {
            Ok(val)
        } else {
            Err(Error::BadCast {
                path: self.path.clone(),
                dtype: "randfloat[]".to_string(),
            })
        }
    }
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct ParameterMap {
    path: String,
    map: BTreeMap<String, ParameterTree>,
}

impl ParameterMap {
    pub fn contains_key(&self, key: &str) -> bool {
        self.map.contains_key(key)
    }

    pub fn get_from_key(&self, key: &str) -> Result<&ParameterTree, Error> {
        self.map.get(key).ok_or(Error::NotFound {
            path: append_path(&self.path, key),
        })
    }

    pub fn get(&self, rel_path: &str) -> Result<&ParameterTree, Error> {
        let mut parts = rel_path.split(".");

        let mut elem = self
            .map
            .get(parts.next().expect("Split cannot return an empty iterator"))
            .ok_or(Error::NotFound {
                path: append_path(&self.path, rel_path),
            })?;

        for part in parts {
            match elem {
                ParameterTree::Node(n) => {
                    elem = n.map.get(part).ok_or(Error::NotFound {
                        path: append_path(&self.path, rel_path),
                    })?;
                }
                ParameterTree::Leaf(_) => {
                    return Err(Error::NotFound {
                        path: append_path(&self.path, rel_path),
                    });
                }
            }
        }

        Ok(elem)
    }

    pub fn get_param(&self, rel_path: &str) -> Result<&Parameter, Error> {
        Ok(self.get(rel_path)?.as_param()?)
    }

    pub fn get_map(&self, rel_path: &str) -> Result<&ParameterMap, Error> {
        Ok(self.get(rel_path)?.as_map()?)
    }

    pub fn iter(&self) -> ParameterMapIter<'_> {
        ParameterMapIter {
            iter: self.map.iter(),
        }
    }

    pub fn resample(&mut self, mut rng: impl Rng) {
        self.resample_inner(&mut rng);
    }

    fn resample_inner<R>(&mut self, rng: &mut R)
    where
        R: Rng,
    {
        for (_, param) in self.map.iter_mut() {
            match param {
                ParameterTree::Node(map) => {
                    map.resample_inner(rng);
                }
                ParameterTree::Leaf(param) => match &mut param.value {
                    ParameterValue::RandFloat(rnd_float) => {
                        rnd_float.sampled = Some(rnd_float.dist.sample(rng));
                        info!(
                            "value={}, sampled={}",
                            rnd_float.val,
                            rnd_float.sampled.unwrap()
                        );
                    }
                    _ => {}
                },
            }
        }
    }
    pub fn resample_perfect(&mut self) {
        for (_, param) in self.map.iter_mut() {
            match param {
                ParameterTree::Node(map) => {
                    map.resample_perfect();
                }
                ParameterTree::Leaf(param) => match &mut param.value {
                    ParameterValue::RandFloat(rnd_float) => {
                        rnd_float.sampled = Some(rnd_float.val);
                    }
                    _ => {}
                },
            }
        }
    }
}

#[derive(Default)]
pub struct ParameterMapIter<'a> {
    iter: btree_map::Iter<'a, String, ParameterTree>,
}

impl<'a> Iterator for ParameterMapIter<'a> {
    type Item = (&'a String, &'a ParameterTree);

    fn next(&mut self) -> Option<Self::Item> {
        self.iter.next()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum ParameterTree {
    Node(ParameterMap),
    Leaf(Parameter),
}

impl Default for ParameterTree {
    fn default() -> Self {
        ParameterTree::Node(ParameterMap::default())
    }
}

impl ParameterTree {
    fn as_param(&self) -> Result<&Parameter, Error> {
        match self {
            Self::Leaf(p) => Ok(p),
            Self::Node(m) => Err(Error::NotAParameter {
                path: m.path.clone(),
            }),
        }
    }

    fn as_map(&self) -> Result<&ParameterMap, Error> {
        match self {
            Self::Node(m) => Ok(m),
            Self::Leaf(p) => Err(Error::NotAMap {
                path: p.path.clone(),
            }),
        }
    }
}

pub fn parse_string(toml_str: String) -> Result<ParameterMap, Error> {
    let table = toml::from_str::<Table>(toml_str.as_str())?;

    parse_table(table)
}

pub fn parse_table(table: Table) -> Result<ParameterMap, Error> {
    parse_table_recursive(table, "".to_string())
}

fn parse_table_recursive(table: Table, root: String) -> Result<ParameterMap, Error> {
    let mut nodes = BTreeMap::new();

    for (key, val) in table.into_iter() {
        let path = append_path(root.as_str(), key.as_str());
        match val {
            Value::Table(val) => {
                if let Ok(value) = val.clone().try_into::<ParameterValue>() {
                    let param = Parameter { path, value };
                    nodes.insert(key, ParameterTree::Leaf(param));
                } else {
                    nodes.insert(key, ParameterTree::Node(parse_table_recursive(val, path)?));
                }
            }
            _ => {
                return Err(Error::BadToml(root));
            }
        }
    }

    Ok(ParameterMap {
        path: root.clone(),
        map: nodes,
    })
}

fn append_path(root: &str, key: &str) -> String {
    format!("{root}.{key}")
}

#[cfg(test)]
mod tests {
    use core::f64;

    use toml::Value;

    use super::*;

    #[test]
    fn test_empty() {
        let str = "".to_string();
        assert_eq!(parse_string(str), Ok(ParameterMap::default()))
    }

    fn test_type(
        expected: Vec<(toml::Value, ParameterValue)>,
        good_value: toml::Value,
        good_type: &str,
        bad_values: Vec<toml::Value>,
        bad_type: &str,
    ) {
        for (val, expected) in expected {
            let str = format!("val = {{ val = {val}, type = \"{good_type}\" }}");
            assert_eq!(
                parse_string(str),
                Ok(ParameterMap {
                    path: "".to_string(),
                    map: BTreeMap::from_iter(vec![(
                        "val".to_string(),
                        ParameterTree::Leaf(Parameter {
                            path: ".val".to_string(),
                            value: expected
                        })
                    )])
                })
            );
        }
        let str = format!("val = {{ val = {good_value}, type = \"badtype\" }}");
        assert_eq!(parse_string(str), Err(Error::BadToml(".val".to_string())));

        let str = format!("val = {{ val = {good_value}, type = \"{bad_type}\" }}",);
        assert_eq!(parse_string(str), Err(Error::BadToml(".val".to_string())));

        for bad_value in bad_values {
            let str = format!("val = {{ val = {bad_value}, type = \"{good_type}\" }}");
            assert_eq!(parse_string(str), Err(Error::BadToml(".val".to_string())));
        }
    }

    #[test]
    fn test_bool() {
        test_type(
            vec![
                (Value::Boolean(true), ParameterValue::Bool { val: true }),
                (Value::Boolean(false), ParameterValue::Bool { val: false }),
            ],
            Value::Boolean(false),
            "bool",
            vec![Value::Float(1.0), Value::Integer(1)],
            "float",
        );
    }

    #[test]
    fn test_int() {
        test_type(
            vec![
                (Value::Integer(-1), ParameterValue::Int { val: -1 }),
                (Value::Integer(1), ParameterValue::Int { val: 1 }),
                (Value::Integer(2), ParameterValue::Int { val: 2 }),
            ],
            Value::Integer(1),
            "int",
            vec![
                Value::Float(1.0),
                Value::Boolean(true),
                Value::String("hello".to_string()),
            ],
            "bool",
        );
    }

    #[test]
    fn test_float() {
        test_type(
            vec![
                (
                    Value::Float(f64::INFINITY),
                    ParameterValue::Float { val: f64::INFINITY },
                ),
                (Value::Float(-1.0), ParameterValue::Float { val: -1.0 }),
                (Value::Integer(1), ParameterValue::Float { val: 1.0 }),
                (Value::Float(1.0), ParameterValue::Float { val: 1.0 }),
                (Value::Float(2.0), ParameterValue::Float { val: 2.0 }),
            ],
            Value::Float(1.0),
            "float",
            vec![Value::Boolean(true), Value::String("hello".to_string())],
            "bool",
        );
    }

    #[test]
    fn test_string() {
        test_type(
            vec![
                (
                    Value::String("hello".to_string()),
                    ParameterValue::String {
                        val: "hello".to_string(),
                    },
                ),
                (
                    Value::String("".to_string()),
                    ParameterValue::String {
                        val: "".to_string(),
                    },
                ),
            ],
            Value::String("hello".to_string()),
            "str",
            vec![Value::Float(1.123), Value::Integer(1), Value::Boolean(true)],
            "bool",
        );
    }

    #[test]
    fn test_good_structure() {
        let str = "hello_float = { val = 1.23, type = \"float\" }
        hello_int = { val = 1, type = \"int\" }
        hello_bool = { val = true, type = \"bool\" }

        [nested]
        hello_int = { val = 1, type = \"int\" }

        [nested.double]
        hello_bool = { val = true, type = \"bool\" }
        ";

        let parsed = parse_string(str.to_string());

        let expected = ParameterMap {
            path: "".to_string(),
            map: BTreeMap::from_iter(vec![
                (
                    "hello_float".to_string(),
                    ParameterTree::Leaf(Parameter {
                        path: ".hello_float".to_string(),
                        value: ParameterValue::Float { val: 1.23 },
                    }),
                ),
                (
                    "hello_int".to_string(),
                    ParameterTree::Leaf(Parameter {
                        path: ".hello_int".to_string(),
                        value: ParameterValue::Int { val: 1 },
                    }),
                ),
                (
                    "hello_bool".to_string(),
                    ParameterTree::Leaf(Parameter {
                        path: ".hello_bool".to_string(),
                        value: ParameterValue::Bool { val: true },
                    }),
                ),
                (
                    "nested".to_string(),
                    ParameterTree::Node(ParameterMap {
                        path: ".nested".to_string(),
                        map: BTreeMap::from_iter(vec![
                            (
                                "hello_int".to_string(),
                                ParameterTree::Leaf(Parameter {
                                    path: ".nested.hello_int".to_string(),
                                    value: ParameterValue::Int { val: 1 },
                                }),
                            ),
                            (
                                "double".to_string(),
                                ParameterTree::Node(ParameterMap {
                                    path: ".nested.double".to_string(),
                                    map: BTreeMap::from_iter(vec![(
                                        "hello_bool".to_string(),
                                        ParameterTree::Leaf(Parameter {
                                            path: ".nested.double.hello_bool".to_string(),
                                            value: ParameterValue::Bool { val: true },
                                        }),
                                    )]),
                                }),
                            ),
                        ]),
                    }),
                ),
            ]),
        };

        assert_eq!(parsed, Ok(expected));
    }

    #[test]
    fn test_array_float() {
        let str = "array = { val = [ 1.0, 2.0, 3 ], type = \"float[]\" }";
        let expected = ParameterMap {
            path: "".to_string(),
            map: BTreeMap::from_iter(vec![(
                "array".to_string(),
                ParameterTree::Leaf(Parameter {
                    path: ".array".to_string(),
                    value: ParameterValue::FloatArray {
                        val: vec![1.0, 2.0, 3.0],
                    },
                }),
            )]),
        };

        assert_eq!(parse_string(str.to_string()), Ok(expected));

        let str = "array = { val = [ ], type = \"float[]\" }";
        let expected = ParameterMap {
            path: "".to_string(),
            map: BTreeMap::from_iter(vec![(
                "array".to_string(),
                ParameterTree::Leaf(Parameter {
                    path: ".array".to_string(),
                    value: ParameterValue::FloatArray { val: vec![] },
                }),
            )]),
        };

        assert_eq!(parse_string(str.to_string()), Ok(expected));

        let str = "array = { val = [ 1.0, 2.0 ], type = \"float\" }";
        assert_eq!(
            parse_string(str.to_string()),
            Err(Error::BadToml(".array".to_string()))
        );

        let str = "array = { val = [ 1.0, 2.0, \"3.0\" ], type = \"float[]\" }";
        assert_eq!(
            parse_string(str.to_string()),
            Err(Error::BadToml(".array".to_string()))
        );
    }

    #[test]
    fn test_rand_float() {
        let str = "rand_float = { val = 1.0, type = \"randfloat\", dist = { type=\"normal\", mean = 1.0, variance = 1.0 } }";
        let expected = ParameterMap {
            path: "".to_string(),
            map: BTreeMap::from_iter(vec![(
                "rand_float".to_string(),
                ParameterTree::Leaf(Parameter {
                    path: ".rand_float".to_string(),
                    value: ParameterValue::RandFloat(RandFloat {
                        val: 1.0,
                        sampled: None,
                        dist: FloatDistribution::Normal {
                            mean: 1.0,
                            variance: 1.0,
                        },
                    }),
                }),
            )]),
        };

        assert_eq!(parse_string(str.to_string()), Ok(expected));

        let str = "rand_float = { val = 1.0, type = \"randfloat\", dist = { type=\"uniform\", min = 1.0, max = 1.0 } }";
        let expected = ParameterMap {
            path: "".to_string(),
            map: BTreeMap::from_iter(vec![(
                "rand_float".to_string(),
                ParameterTree::Leaf(Parameter {
                    path: ".rand_float".to_string(),
                    value: ParameterValue::RandFloat(RandFloat {
                        val: 1.0,
                        sampled: None,
                        dist: FloatDistribution::Uniform { min: 1.0, max: 1.0 },
                    }),
                }),
            )]),
        };

        assert_eq!(parse_string(str.to_string()), Ok(expected));
    }
}
