use core::fmt;

use itertools::join;
use thiserror::Error;

#[derive(Error, Clone, Debug, PartialEq, Eq, Default)]
#[error("Invalid Path")]
pub struct PathError {}

pub fn validate_path(path: &str) -> bool {
    path.starts_with('/')
        && path
            .chars()
            .all(|c| c.is_alphanumeric() || c == '_' || c == '/')
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Path {
    path: String,
    is_root: bool,
}

impl Path {
    pub fn from_str(path: &str) -> Result<Self, PathError> {
        if validate_path(path) {
            let path = format!("/{}", join(Self::split_parts(path), "/"));
            let is_root = path == "/";
            Ok(Path { path, is_root })
        } else {
            Err(PathError::default())
        }
    }

    pub fn as_str(&self) -> &str {
        &self.path
    }

    pub fn path(&self) -> &str {
        &self.path
    }

    pub fn iter_parts(&self) -> impl Iterator<Item = &str> {
        Self::split_parts(&self.path)
    }

    pub fn split_parts(path: &str) -> impl Iterator<Item = &str> {
        path.split('/').filter(|p| p.len() > 0)
    }

    pub fn is_root(&self) -> bool {
        self.is_root
    }
}

impl From<&str> for Path {
    fn from(value: &str) -> Self {
        Path::from_str(value).expect("Bad path")
    }
}

impl From<String> for Path {
    fn from(value: String) -> Self {
        Path::from_str(value.as_str()).expect("Bad path")
    }
}

impl From<Path> for String {
    fn from(value: Path) -> Self {
        value.path.clone()
    }
}

impl<'a> From<&'a Path> for &'a str {
    fn from(value: &'a Path) -> Self {
        value.path.as_str()
    }
}

impl fmt::Display for Path {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.as_str())
    }
}

#[cfg(test)]
mod tests {
    use crate::utils::path::{validate_path, Path, PathError};

    #[test]
    fn test_validate_path() {
        assert!(validate_path("/a"));
        assert!(validate_path("/a/b"));
        assert!(validate_path("/a/b/c"));
        assert!(validate_path("/a/b/c_1"));
        assert!(validate_path("/a__1/b/c_1"));
        assert!(validate_path("/a//b"));
        assert!(validate_path("/a/b/"));
        assert!(validate_path("/"));

        assert!(!validate_path("a"));
        assert!(!validate_path("a/b"));
        assert!(!validate_path("/a/b /c"));
        assert!(!validate_path("/a!/b"));
    }

    #[test]
    fn test_path_from_str() {
        assert!(Path::from_str("/a").is_ok());
        assert!(Path::from_str("/a/b").is_ok());
        assert!(Path::from_str("/a/b/c").is_ok());
        assert!(Path::from_str("/a/b/c_1").is_ok());
        assert!(Path::from_str("/a__1/b/c_1").is_ok());
        assert!(Path::from_str("/a//b").is_ok());
        assert!(Path::from_str("/a/b/").is_ok());
        assert!(Path::from_str("/").is_ok());

        assert_eq!(Path::from_str("a"), Err(PathError::default()));
        assert_eq!(Path::from_str("a/b"), Err(PathError::default()));
        assert_eq!(Path::from_str("/a/b /c"), Err(PathError::default()));
        assert_eq!(Path::from_str("/a!/b"), Err(PathError::default()));
    }

    #[test]
    fn test_from_trait_str() {
        let p: Path = "/abc".into();
        
        assert_eq!(p.as_str(), "/abc")
    }

    #[test]
    #[should_panic]
    fn test_from_trait_str_panic() {
        let _: Path = "abc".into();
    }

    #[test]
    fn test_path_iter_parts() {
        let parts: Vec<_> = Path::from_str("/a/b/c")
            .unwrap()
            .iter_parts()
            .map(|p| p.to_string())
            .collect();
        assert_eq!(parts, vec!["a", "b", "c"]);

        let parts: Vec<_> = Path::from_str("///a///b////c////")
            .unwrap()
            .iter_parts()
            .map(|p| p.to_string())
            .collect();
        assert_eq!(parts, vec!["a", "b", "c"]);

        let parts: Vec<_> = Path::from_str("/a")
            .unwrap()
            .iter_parts()
            .map(|p| p.to_string())
            .collect();
        assert_eq!(parts, vec!["a"]);
    }
}
