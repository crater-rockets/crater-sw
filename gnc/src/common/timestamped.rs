use crate::{Instant, InstantU64};

#[derive(Debug, Clone, Copy)]
pub struct Timestamped<T> {
    pub t: Instant,
    pub v: T,
}

impl<T> Timestamped<T> {
    pub fn new(t: Instant, v: T) -> Self {
        Timestamped { t, v }
    }

    pub fn from_microseconds(t: u64, v: T) -> Self {
        Timestamped {
            t: Instant(InstantU64::from_ticks(t)),
            v,
        }
    }
}

pub type Ts<T> = Timestamped<T>;
