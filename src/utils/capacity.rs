use std::num::NonZero;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Capacity {
    Unbounded,
    Bounded(NonZero<usize>),
}

impl<T> From<T> for Capacity
where
    T: Into<usize>,
{
    fn from(value: T) -> Self {
        let value: usize = value.into();
        if value == 0 {
            Capacity::Unbounded
        } else {
            Capacity::Bounded(NonZero::new(value).unwrap())
        }
    }
}
