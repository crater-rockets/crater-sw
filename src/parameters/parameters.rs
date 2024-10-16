use super::deser::{self};
use crate::{
    parameters::deser::parse_str,
    utils::path::{Path, PathError},
};
use itertools::join;
use std::{
    collections::{btree_map, BTreeMap},
    fmt::Display,
    mem,
    sync::{Arc, Mutex},
};
use thiserror::Error;

#[derive(Debug, Clone, Error, PartialEq, Eq)]
pub enum Error {
    #[error("Trying to set a parameter whose parent is not a map")]
    NonMapParent,

    #[error("Cannot overwrite root parameter")]
    RootOverwrite,

    #[error("Error parsing parameter from toml")]
    Toml(#[from] deser::Error),

    #[error("Invalid path")]
    Path(#[from] PathError),

    #[error("No parameter found for path '{0}'")]
    NotFound(Path),

    #[error("Request type '{0}' for parameter '{1}', but is a '{2}")]
    TypeMismatch(String, Path, String),
}

#[derive(Debug, Clone, PartialEq)]
pub enum Parameter {
    Bool(bool),

    U8(u8),
    U16(u16),
    U32(u32),
    U64(u64),

    I8(i8),
    I16(i16),
    I32(i32),
    I64(i64),

    F32(f32),
    F64(f64),

    String(String),
    List(Vec<Parameter>),
    Map(BTreeMap<String, Parameter>),
}

impl Parameter {
    pub fn iter(&self) -> ParameterIter<'_> {
        ParameterIter {
            iter: if let Parameter::Map(m) = &self {
                m.iter()
            } else {
                Default::default()
            },
            parent: None,
            path: "".to_string(),
        }
    }

    fn type_string(&self) -> &str {
        match self {
            Parameter::Bool(_) => "bool",
            Parameter::U8(_) => "u8",
            Parameter::U16(_) => "u16",
            Parameter::U32(_) => "u32",
            Parameter::U64(_) => "u64",
            Parameter::I8(_) => "i8",
            Parameter::I16(_) => "i16",
            Parameter::I32(_) => "i32",
            Parameter::I64(_) => "i64",
            Parameter::F32(_) => "f32",
            Parameter::F64(_) => "f64",
            Parameter::String(_) => "string",
            Parameter::List(_) => "list",
            Parameter::Map(_) => "map",
        }
    }

    pub fn as_bool(&self) -> Option<&bool> {
        if let Parameter::Bool(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_u8(&self) -> Option<&u8> {
        if let Parameter::U8(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_u16(&self) -> Option<&u16> {
        if let Parameter::U16(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_u32(&self) -> Option<&u32> {
        if let Parameter::U32(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_u64(&self) -> Option<&u64> {
        if let Parameter::U64(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_i8(&self) -> Option<&i8> {
        if let Parameter::I8(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_i16(&self) -> Option<&i16> {
        if let Parameter::I16(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_i32(&self) -> Option<&i32> {
        if let Parameter::I32(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_i64(&self) -> Option<&i64> {
        if let Parameter::I64(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_f32(&self) -> Option<&f32> {
        if let Parameter::F32(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_f64(&self) -> Option<&f64> {
        if let Parameter::F64(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_string(&self) -> Option<&String> {
        if let Parameter::String(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_list(&self) -> Option<&Vec<Parameter>> {
        if let Parameter::List(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_map(&self) -> Option<&BTreeMap<String, Parameter>> {
        if let Parameter::Map(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn is_bool(&self) -> bool {
        if let Parameter::Bool(_) = self {
            true
        } else {
            false
        }
    }

    pub fn is_u8(&self) -> bool {
        if let Parameter::U8(_) = self {
            true
        } else {
            false
        }
    }

    pub fn is_u16(&self) -> bool {
        if let Parameter::U16(_) = self {
            true
        } else {
            false
        }
    }

    pub fn is_u32(&self) -> bool {
        if let Parameter::U32(_) = self {
            true
        } else {
            false
        }
    }

    pub fn is_u64(&self) -> bool {
        if let Parameter::U64(_) = self {
            true
        } else {
            false
        }
    }

    pub fn is_i8(&self) -> bool {
        if let Parameter::I8(_) = self {
            true
        } else {
            false
        }
    }

    pub fn is_i16(&self) -> bool {
        if let Parameter::I16(_) = self {
            true
        } else {
            false
        }
    }

    pub fn is_i32(&self) -> bool {
        if let Parameter::I32(_) = self {
            true
        } else {
            false
        }
    }

    pub fn is_i64(&self) -> bool {
        if let Parameter::I64(_) = self {
            true
        } else {
            false
        }
    }

    pub fn is_f32(&self) -> bool {
        if let Parameter::F32(_) = self {
            true
        } else {
            false
        }
    }

    pub fn is_f64(&self) -> bool {
        if let Parameter::F64(_) = self {
            true
        } else {
            false
        }
    }

    pub fn is_string(&self) -> bool {
        if let Parameter::String(_) = self {
            true
        } else {
            false
        }
    }

    pub fn is_list(&self) -> bool {
        if let Parameter::List(_) = self {
            true
        } else {
            false
        }
    }

    pub fn is_map(&self) -> bool {
        if let Parameter::Map(_) = self {
            true
        } else {
            false
        }
    }
}

impl Display for Parameter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Parameter::Bool(v) => write!(f, "{}", v),
            Parameter::U8(v) => write!(f, "{}", v),
            Parameter::U16(v) => write!(f, "{}", v),
            Parameter::U32(v) => write!(f, "{}", v),
            Parameter::U64(v) => write!(f, "{}", v),
            Parameter::I8(v) => write!(f, "{}", v),
            Parameter::I16(v) => write!(f, "{}", v),
            Parameter::I32(v) => write!(f, "{}", v),
            Parameter::I64(v) => write!(f, "{}", v),
            Parameter::F32(v) => write!(f, "{}", v),
            Parameter::F64(v) => write!(f, "{}", v),
            Parameter::String(v) => write!(f, "{}", v),
            Parameter::List(v) => write!(f, "[{}]", join(v.iter().map(|v| v.to_string()), ", ")),
            Parameter::Map(v) => {
                write!(
                    f,
                    "{{{}}}",
                    join(v.iter().map(|(k, v)| format!("'{}': {}", k, v)), ", ")
                )
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct ParameterService {
    inner: Arc<Mutex<ParameterServiceInner>>,
}

#[derive(Debug, Clone)]
struct ParameterServiceInner {
    root: Parameter,
}

impl Default for ParameterService {
    fn default() -> Self {
        ParameterService {
            inner: Arc::new(Mutex::new(ParameterServiceInner {
                root: Parameter::Map(BTreeMap::default()),
            })),
        }
    }
}

impl ParameterService {
    pub fn from_toml(toml: &str) -> Result<Self, Error> {
        let parsed: Vec<(String, Parameter)> = parse_str(toml)?;

        Self::from_list(parsed)
    }

    fn from_list(value: Vec<(String, Parameter)>) -> Result<Self, Error> {
        let mut ps = ParameterService::default();
        for (path, val) in value {
            ps.set(&path.into(), val)?;
        }

        Ok(ps)
    }

    #[allow(dead_code)]
    fn from_root(root: Parameter) -> Self {
        ParameterService {
            inner: Arc::new(Mutex::new(ParameterServiceInner { root })),
        }
    }

    pub fn get(&self, path: &Path) -> Option<Parameter> {
        let mut root = &self.inner.lock().unwrap().root;

        for part in path.iter_parts() {
            match root {
                Parameter::Map(m) => {
                    root = m.get(part)?;
                }
                _ => {
                    return None;
                }
            }
        }

        Some(root.clone())
    }

    pub fn get_bool(&self, path: &str) -> Result<bool, Error> {
        let path = Path::from_str(path)?;
        let param = self.get(&path).ok_or(Error::NotFound(path.clone()))?;
        Ok(*param.as_bool().ok_or(Error::TypeMismatch(
            "bool".to_string(),
            path,
            param.type_string().to_string(),
        ))?)
    }

    pub fn get_u8(&self, path: &str) -> Result<u8, Error> {
        let path = Path::from_str(path)?;
        let param = self.get(&path).ok_or(Error::NotFound(path.clone()))?;
        Ok(*param.as_u8().ok_or(Error::TypeMismatch(
            "u8".to_string(),
            path,
            param.type_string().to_string(),
        ))?)
    }

    pub fn get_u16(&self, path: &str) -> Result<u16, Error> {
        let path = Path::from_str(path)?;
        let param = self.get(&path).ok_or(Error::NotFound(path.clone()))?;
        Ok(*param.as_u16().ok_or(Error::TypeMismatch(
            "u16".to_string(),
            path,
            param.type_string().to_string(),
        ))?)
    }

    pub fn get_u32(&self, path: &str) -> Result<u32, Error> {
        let path = Path::from_str(path)?;
        let param = self.get(&path).ok_or(Error::NotFound(path.clone()))?;
        Ok(*param.as_u32().ok_or(Error::TypeMismatch(
            "u32".to_string(),
            path,
            param.type_string().to_string(),
        ))?)
    }

    pub fn get_u64(&self, path: &str) -> Result<u64, Error> {
        let path = Path::from_str(path)?;
        let param = self.get(&path).ok_or(Error::NotFound(path.clone()))?;
        Ok(*param.as_u64().ok_or(Error::TypeMismatch(
            "u64".to_string(),
            path,
            param.type_string().to_string(),
        ))?)
    }

    pub fn get_i8(&self, path: &str) -> Result<i8, Error> {
        let path = Path::from_str(path)?;
        let param = self.get(&path).ok_or(Error::NotFound(path.clone()))?;
        Ok(*param.as_i8().ok_or(Error::TypeMismatch(
            "i8".to_string(),
            path,
            param.type_string().to_string(),
        ))?)
    }

    pub fn get_i16(&self, path: &str) -> Result<i16, Error> {
        let path = Path::from_str(path)?;
        let param = self.get(&path).ok_or(Error::NotFound(path.clone()))?;
        Ok(*param.as_i16().ok_or(Error::TypeMismatch(
            "i16".to_string(),
            path,
            param.type_string().to_string(),
        ))?)
    }

    pub fn get_i32(&self, path: &str) -> Result<i32, Error> {
        let path = Path::from_str(path)?;
        let param = self.get(&path).ok_or(Error::NotFound(path.clone()))?;
        Ok(*param.as_i32().ok_or(Error::TypeMismatch(
            "i32".to_string(),
            path,
            param.type_string().to_string(),
        ))?)
    }

    pub fn get_i64(&self, path: &str) -> Result<i64, Error> {
        let path = Path::from_str(path)?;
        let param = self.get(&path).ok_or(Error::NotFound(path.clone()))?;
        Ok(*param.as_i64().ok_or(Error::TypeMismatch(
            "i64".to_string(),
            path,
            param.type_string().to_string(),
        ))?)
    }

    pub fn get_f32(&self, path: &str) -> Result<f32, Error> {
        let path = Path::from_str(path)?;
        let param = self.get(&path).ok_or(Error::NotFound(path.clone()))?;
        Ok(*param.as_f32().ok_or(Error::TypeMismatch(
            "f32".to_string(),
            path,
            param.type_string().to_string(),
        ))?)
    }

    pub fn get_f64(&self, path: &str) -> Result<f64, Error> {
        let path = Path::from_str(path)?;
        let param = self.get(&path).ok_or(Error::NotFound(path.clone()))?;
        Ok(*param.as_f64().ok_or(Error::TypeMismatch(
            "f64".to_string(),
            path,
            param.type_string().to_string(),
        ))?)
    }

    pub fn get_string(&self, path: &str) -> Result<String, Error> {
        let path = Path::from_str(path)?;
        let param = self.get(&path).ok_or(Error::NotFound(path.clone()))?;
        Ok(param
            .as_string()
            .ok_or(Error::TypeMismatch(
                "String".to_string(),
                path,
                param.type_string().to_string(),
            ))?
            .clone())
    }

    pub fn set(&mut self, path: &Path, val: Parameter) -> Result<Option<Parameter>, Error> {
        if path.is_root() {
            return Err(Error::RootOverwrite);
        }

        let mut root = &mut self.inner.lock().unwrap().root;

        for part in Self::skip_last(path.iter_parts()) {
            root = match root {
                Parameter::Map(m) => {
                    if m.contains_key(part) {
                        m.get_mut(part).unwrap()
                    } else {
                        m.insert(part.to_string(), Parameter::Map(BTreeMap::new()));
                        m.get_mut(part).unwrap()
                    }
                }
                _ => {
                    return Err(Error::NonMapParent);
                }
            };
        }

        if let Parameter::Map(root) = root {
            Ok(root.insert(path.iter_parts().last().unwrap().to_string(), val))
        } else {
            return Err(Error::NonMapParent);
        }
    }

    #[allow(dead_code)]
    fn as_vec(&self) -> Vec<(String, Parameter)> {
        self.inner
            .lock()
            .unwrap()
            .iter()
            .map(|(p, par)| (p, par.clone()))
            .collect::<Vec<_>>()
    }

    fn skip_last<T>(mut iter: impl Iterator<Item = T>) -> impl Iterator<Item = T> {
        let last = iter.next();
        iter.scan(last, |state, item| std::mem::replace(state, Some(item)))
    }
}

impl ParameterServiceInner {
    pub fn iter(&self) -> ParameterIter<'_> {
        self.root.iter()
    }
}

impl Display for ParameterService {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let inner = self.inner.lock().unwrap();
        for (path, v) in inner.iter() {
            write!(f, "{}: {}  = {}", path, v.type_string(), v)?;
        }

        Ok(())
    }
}

#[derive(Default)]
pub struct ParameterIter<'a> {
    iter: btree_map::Iter<'a, String, Parameter>,
    parent: Option<Box<ParameterIter<'a>>>,
    path: String,
}

impl<'a> Iterator for ParameterIter<'a> {
    type Item = (String, &'a Parameter);
    fn next(&mut self) -> Option<Self::Item> {
        match self.iter.next() {
            None => match self.parent.take() {
                Some(p) => {
                    *self = *p;
                    self.next()
                }
                None => None,
            },
            Some((name, Parameter::Map(m))) => {
                let path = format!("{}/{}", self.path, name);
                *self = ParameterIter {
                    iter: m.iter(),
                    parent: Some(Box::new(mem::take(self))),
                    path,
                };
                self.next()
            }

            Some((name, param)) => Some((format!("{}/{}", self.path, name), param)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds the following parameter tree:
    /// ```
    /// {
    ///     "a1": F32(1.23)
    ///     "a2": String("hello param")
    ///     "a3": {
    ///         "b1": {
    ///             "c1": I32(123),
    ///             "c2": I32(-123)
    ///         },
    ///         "b2": F64(1.23)
    ///     }
    /// }
    /// ```
    fn build_params() -> Parameter {
        Parameter::Map(BTreeMap::from([
            ("a1".to_string(), Parameter::F32(1.23)),
            (
                "a2".to_string(),
                Parameter::String("hello param".to_string()),
            ),
            (
                "a3".to_string(),
                Parameter::Map(BTreeMap::from([
                    (
                        "b1".to_string(),
                        Parameter::Map(BTreeMap::from([
                            ("c1".to_string(), Parameter::I32(123)),
                            ("c2".to_string(), Parameter::I32(-123)),
                        ])),
                    ),
                    ("b2".to_string(), Parameter::F64(1.23)),
                ])),
            ),
        ]))
    }

    #[test]
    fn test_get() {
        let root = build_params();

        let ps = ParameterService::from_root(root.clone());

        assert_eq!(ps.get(&Path::from_str("/").unwrap()), Some(root));

        assert_eq!(
            ps.get(&Path::from_str("/a1").unwrap()),
            Some(Parameter::F32(1.23))
        );

        assert_eq!(
            ps.get(&Path::from_str("/a3/b1/c1").unwrap()),
            Some(Parameter::I32(123))
        );

        assert_eq!(
            ps.get(&Path::from_str("/a3/b1/").unwrap()),
            Some(Parameter::Map(BTreeMap::from([
                ("c1".to_string(), Parameter::I32(123)),
                ("c2".to_string(), Parameter::I32(-123)),
            ])))
        );

        assert_eq!(ps.get(&Path::from_str("/a99").unwrap()), None);
        assert_eq!(ps.get(&Path::from_str("/a3/b1/c1/d1").unwrap()), None);

        assert_eq!(ps.get(&Path::from_str("/a3/b1/c99").unwrap()), None);
    }

    #[test]
    fn test_set() {
        // root = {
        //     "a1": F32(1.23)
        //     "a2": String("hello param")
        //     "a3": {
        //         "b1": {
        //             "c1": I32(123),
        //             "c2": I32(-123)
        //         },
        //         "b2": F64(1.23)
        //     }
        // }
        let mut ps = ParameterService::from_root(build_params());

        let mut flattened: BTreeMap<&str, Parameter> = BTreeMap::from([
            ("/a1", Parameter::F32(1.23)),
            ("/a2", Parameter::String("hello param".to_string())),
            ("/a3/b1/c1", Parameter::I32(123)),
            ("/a3/b1/c2", Parameter::I32(-123)),
            ("/a3/b2", Parameter::F64(1.23)),
        ]);

        fn check_parameters(ps: &ParameterService, flattened: &BTreeMap<&str, Parameter>) {
            for (path, param) in flattened.iter() {
                assert_eq!(ps.get(&(*path).into()), Some(param.clone()));
            }
        }

        check_parameters(&ps, &flattened);

        assert_eq!(
            ps.set(
                &"/a4".into(),
                Parameter::List(vec![Parameter::I8(1), Parameter::I8(2)]),
            ),
            Ok(None)
        );
        flattened.insert(
            "/a4",
            Parameter::List(vec![Parameter::I8(1), Parameter::I8(2)]),
        );
        check_parameters(&ps, &flattened);

        assert_eq!(ps.set(&"/a3/b1/c3".into(), Parameter::Bool(true)), Ok(None));
        flattened.insert("/a3/b1/c3", Parameter::Bool(true));
        check_parameters(&ps, &flattened);

        assert_eq!(
            ps.set(&"/a3/b1/c4/d1/e1".into(), Parameter::Bool(true)),
            Ok(None)
        );
        flattened.insert("/a3/b1/c4/d1/e1", Parameter::Bool(true));
        check_parameters(&ps, &flattened);

        assert_eq!(
            ps.set(&"/a3/b1/c1".into(), Parameter::Bool(true)),
            Ok(Some(Parameter::I32(123)))
        );
        flattened.insert("/a3/b1/c1", Parameter::Bool(true));
        check_parameters(&ps, &flattened);

        assert_eq!(
            ps.set(&"/a3/b1/c1/d1".into(), Parameter::Bool(true)),
            Err(Error::NonMapParent)
        );
        check_parameters(&ps, &flattened);

        assert_eq!(
            ps.set(&"/".into(), Parameter::Bool(true)),
            Err(Error::RootOverwrite)
        );
        check_parameters(&ps, &flattened);
    }

    #[test]
    fn test_iter() {
        let ps = ParameterService::from_root(build_params());

        let flattened: Vec<(String, Parameter)> = vec![
            ("/a1".to_string(), Parameter::F32(1.23)),
            (
                "/a2".to_string(),
                Parameter::String("hello param".to_string()),
            ),
            ("/a3/b1/c1".to_string(), Parameter::I32(123)),
            ("/a3/b1/c2".to_string(), Parameter::I32(-123)),
            ("/a3/b2".to_string(), Parameter::F64(1.23)),
        ];

        assert_eq!(ps.as_vec(), flattened);
    }

    #[test]
    fn test_from_toml() -> Result<(), Error> {
        let ps = ParameterService::from_toml(
            "a1 = {val=1.23, dtype=\"f32\"}
                a2 = {val=\"hello param\", dtype=\"string\"}

                [a3]
                b2 = {val=1.23, dtype=\"f64\"}
                [a3.b1]
                c1 = {val=123, dtype=\"i32\"}
                c2 = {val=-123, dtype=\"i32\"}

                ",
        )?;

        let flattened: Vec<(String, Parameter)> = vec![
            ("/a1".to_string(), Parameter::F32(1.23)),
            (
                "/a2".to_string(),
                Parameter::String("hello param".to_string()),
            ),
            ("/a3/b1/c1".to_string(), Parameter::I32(123)),
            ("/a3/b1/c2".to_string(), Parameter::I32(-123)),
            ("/a3/b2".to_string(), Parameter::F64(1.23)),
        ];

        assert_eq!(ps.as_vec(), flattened);

        Ok(())
    }
}
