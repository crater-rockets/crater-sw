use serde::Deserialize;
use thiserror::Error;
use toml::{Table, Value};

use super::Parameter;
use anyhow::Result;

#[derive(Debug, Clone, Error, PartialEq, Eq)]
pub enum Error {
    #[error("Parameter toml does not have the right structure")]
    BadStructure,

    #[error("Unexpected Value variant encountered")]
    UnexpectedValue,

    #[error("Cannot convert value to '{0}'")]
    BadConversion(String),

    #[error("Invalid type: '{0}'")]
    InvalidType(String),

    #[error("Error deserializing parameters")]
    Deserialize(#[from] toml::de::Error),
}

#[derive(Debug, Clone, Deserialize)]
struct ParameterDef {
    val: Value,
    dtype: String,
}

pub(super) fn parse_str(toml_str: &str) -> Result<Vec<(String, Parameter)>, Error> {
    let table = toml::from_str::<Table>(toml_str)?;

    parse_table(table, "")
}

pub(super) fn parse_table(table: Table, path: &str) -> Result<Vec<(String, Parameter)>, Error> {
    let mut params: Vec<(String, Parameter)> = vec![];
    if let Ok(def) = table.clone().try_into::<ParameterDef>() {
        if path == "" {
            // Root must be a map, not a scalar / list
            return Err(Error::BadStructure);
        }
        params.push((path.to_string(), value_to_parameter(def.val, &def.dtype)?));
    } else {
        for (k, v) in table.into_iter() {
            let nested = format!("{path}/{k}");
            if let Value::Table(t) = v {
                params.append(&mut parse_table(t, nested.as_str())?);
            } else {
                return Err(Error::BadStructure);
            }
        }
    }
    Ok(params)
}

fn value_to_parameter(value: Value, dtype: &str) -> Result<Parameter, Error> {
    match value {
        Value::Array(arr) => {
            let mut out = Vec::with_capacity(arr.len());
            for v in arr {
                out.push(value_to_parameter(v, dtype)?);
            }

            Ok(Parameter::List(out))
        }
        Value::Table(_) => Err(Error::UnexpectedValue),
        value => match dtype {
            "bool" => Ok(Parameter::Bool(
                value
                    .as_bool()
                    .ok_or(Error::BadConversion("bool".to_string()))?,
            )),

            "u8" => Ok(Parameter::U8(
                value
                    .as_integer()
                    .ok_or(Error::BadConversion("u8".to_string()))? as u8,
            )),
            "u16" => Ok(Parameter::U16(
                value
                    .as_integer()
                    .ok_or(Error::BadConversion("u16".to_string()))? as u16,
            )),
            "u32" => Ok(Parameter::U32(
                value
                    .as_integer()
                    .ok_or(Error::BadConversion("u32".to_string()))? as u32,
            )),
            "u64" => Ok(Parameter::U64(
                value
                    .as_integer()
                    .ok_or(Error::BadConversion("u64".to_string()))? as u64,
            )),

            "i8" => Ok(Parameter::I8(
                value
                    .as_integer()
                    .ok_or(Error::BadConversion("i8".to_string()))? as i8,
            )),
            "i16" => Ok(Parameter::I16(
                value
                    .as_integer()
                    .ok_or(Error::BadConversion("i16".to_string()))? as i16,
            )),
            "i32" => Ok(Parameter::I32(
                value
                    .as_integer()
                    .ok_or(Error::BadConversion("i32".to_string()))? as i32,
            )),
            "i64" => Ok(Parameter::I64(
                value
                    .as_integer()
                    .ok_or(Error::BadConversion("i64".to_string()))? as i64,
            )),
            "f32" => Ok(Parameter::F32(
                value
                    .as_float()
                    .or(value.as_integer().map(|v| v as f64))
                    .ok_or(Error::BadConversion("f32".to_string()))? as f32,
            )),
            "f64" => Ok(Parameter::F64(
                value
                    .as_float()
                    .or(value.as_integer().map(|v| v as f64))
                    .ok_or(Error::BadConversion("f64".to_string()))?,
            )),

            "string" => Ok(Parameter::String(
                value
                    .as_str()
                    .ok_or(Error::BadConversion("string".to_string()))?
                    .to_string(),
            )),

            dtype => Err(Error::InvalidType(dtype.to_string())),
        },
    }
}

#[cfg(test)]
mod test {
    use super::*;

    use anyhow::Result;

    #[test]
    fn test_from_string() -> Result<()> {
        assert_eq!(
            parse_str(
                "a1 = {val=1.23, dtype=\"f32\"}
                a2 = {val=\"hello param\", dtype=\"string\"}

                [a3]
                b2 = {val=1.23, dtype=\"f64\"}
                [a3.b1]
                c1 = {val=123, dtype=\"i32\"}
                c2 = {val=-123, dtype=\"i32\"}

                "
            ),
            Ok(vec![
                ("/a1".to_string(), Parameter::F32(1.23)),
                (
                    "/a2".to_string(),
                    Parameter::String("hello param".to_string()),
                ),
                ("/a3/b1/c1".to_string(), Parameter::I32(123)),
                ("/a3/b1/c2".to_string(), Parameter::I32(-123)),
                ("/a3/b2".to_string(), Parameter::F64(1.23)),
            ])
        );

        assert_eq!(
            parse_str(
                "a1 = {val=1.23, dtype=\"f32\"}
                [a2]
                val=1.23
                dtype=\"f32\"
                "
            ),
            Ok(vec![
                ("/a1".to_string(), Parameter::F32(1.23)),
                ("/a2".to_string(), Parameter::F32(1.23),),
            ])
        );

        Ok(())
    }

    #[test]
    fn test_integer_to_float() -> Result<()> {
        assert_eq!(
            parse_str("a1 = {val=1, dtype=\"f32\"}"),
            Ok(vec![("/a1".to_string(), Parameter::F32(1.0))])
        );

        assert_eq!(
            parse_str("a1 = {val=1, dtype=\"f64\"}"),
            Ok(vec![("/a1".to_string(), Parameter::F64(1.0))])
        );
        Ok(())
    }

    #[test]
    fn test_lists() -> Result<()> {
        assert_eq!(
            parse_str("a1 = {val=[], dtype=\"i32\"}"),
            Ok(vec![("/a1".to_string(), Parameter::List(vec![]))])
        );

        assert_eq!(
            parse_str("a1 = {val=[1,2,3], dtype=\"i32\"}"),
            Ok(vec![(
                "/a1".to_string(),
                Parameter::List(vec![
                    Parameter::I32(1),
                    Parameter::I32(2),
                    Parameter::I32(3)
                ])
            )])
        );

        assert_eq!(
            parse_str("a1 = {val=[\"a\", \"b\", \"c\"], dtype=\"string\"}"),
            Ok(vec![(
                "/a1".to_string(),
                Parameter::List(vec![
                    Parameter::String("a".to_string()),
                    Parameter::String("b".to_string()),
                    Parameter::String("c".to_string()),
                ])
            )])
        );
        Ok(())
    }

    #[test]
    fn test_bad_dtype() -> Result<()> {
        assert_eq!(
            parse_str("a1 = {val=1.23, dtype=\"h32\"}"),
            Err(Error::InvalidType("h32".to_string()))
        );
        Ok(())
    }

    #[test]
    fn test_bad_conversion() -> Result<()> {
        assert_eq!(
            parse_str("a1 = {val=1.23, dtype=\"i32\"}"),
            Err(Error::BadConversion("i32".to_string()))
        );

        assert_eq!(
            parse_str("a1 = {val=\"abcd\", dtype=\"i32\"}"),
            Err(Error::BadConversion("i32".to_string()))
        );

        assert_eq!(
            parse_str("a1 = {val=[1.1, 2.2], dtype=\"i32\"}"),
            Err(Error::BadConversion("i32".to_string()))
        );

        Ok(())
    }

    #[test]
    fn test_etero_list() -> Result<()> {
        assert_eq!(
            parse_str("a1 = {val=[1, 2.1, 3.2], dtype=\"f32\"}"),
            Ok(vec![(
                "/a1".to_string(),
                Parameter::List(vec![
                    Parameter::F32(1.0),
                    Parameter::F32(2.1),
                    Parameter::F32(3.2),
                ])
            )])
        );

        assert_eq!(
            parse_str("a1 = {val=[1, 2.0, 3], dtype=\"i32\"}"),
            Err(Error::BadConversion("i32".to_string()))
        );

        assert_eq!(
            parse_str("a1 = {val=[1, 2, \"3\"], dtype=\"i32\"}"),
            Err(Error::BadConversion("i32".to_string()))
        );

        Ok(())
    }

    #[test]
    fn test_bad_structure() -> Result<()> {
        assert_eq!(
            parse_str(
                "a1 = {val=1.23, dtype=\"f32\"}
                a2 = 2
                "
            ),
            Err(Error::BadStructure)
        );

        assert_eq!(
            parse_str(
                "val = 1.23
                dtype = \"f32\"
                "
            ),
            Err(Error::BadStructure)
        );

        assert_eq!(
            parse_str(
                "a1 = {val=1.23, typo=\"f32\"}
                "
            ),
            Err(Error::BadStructure)
        );

        assert!(parse_str(
            "a1 = {val=1.23, dtype=\"f32\"}

            [a1]
            c1 = {val=123, dtype=\"i32\"}

            "
        )
        .is_err());

        Ok(())
    }
}
