use thiserror::Error;

#[derive(Error, Clone, Debug, PartialEq, Eq, Default)]
#[error("Invalid Path")]
pub struct PathError {}

pub fn split_parts(path: &str) -> Option<Vec<String>> {
    if !path.starts_with('/') {
        return None;
    }

    let mut parts: Vec<String> = path.split("/").skip(1).map(|v| v.to_string()).collect();

    for v in parts.iter_mut() {
        if v.len() == 0 || !v.chars().all(|c| c.is_alphanumeric() || c == '_') {
            return None;
        }
    }

    Some(parts)
}

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
            Ok(Path {
                path: path.to_string(),
                is_root: path.split('/').filter(|p| p.len() > 0).count() == 0,
            })
        } else {
            Err(PathError::default())
        }
    }

    pub fn path(&self) -> &str {
        &self.path
    }

    pub fn iter_parts(&self) -> impl Iterator<Item = &str> {
        self.path.split('/').filter(|p| p.len() > 0)
    }

    pub fn is_root(&self) -> bool {
        self.is_root
    }
}

impl ToString for Path {
    fn to_string(&self) -> String {
        self.path.to_string()
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
