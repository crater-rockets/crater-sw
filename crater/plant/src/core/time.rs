use chrono::{TimeDelta, Utc};
use crater_core::time::{Clock, Instant, UtcInstant};

#[derive(Debug, Clone, Default)]
pub struct SystemClock;

impl Clock for SystemClock {
    fn utc(&self) -> Option<UtcInstant> {
        Some(UtcInstant { utc: Utc::now() })
    }

    fn monotonic(&self) -> Instant {
        Instant {
            delta: TimeDelta::from_std(std::time::Instant::now().elapsed()).unwrap(),
        }
    }
}
