use crate::Instant;

pub trait Hal {
    fn now() -> Instant;
}
