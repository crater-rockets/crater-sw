use crate::Instant;

pub trait Hal {
    fn system_time(&self) -> Instant;


}

pub mod channel;