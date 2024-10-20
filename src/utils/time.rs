use std::ops::{Add, AddAssign, Sub, SubAssign};

use chrono::{DateTime, TimeDelta, Utc};

pub trait Clock {
    fn realtime(&self) -> SystemClock;
    fn monotonic(&self) -> Instant;
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Ord, Eq, Hash)]
pub struct Instant {
    delta: TimeDelta,
}

impl Instant {
    pub fn elapsed(&self) -> TimeDelta {
        self.delta
    }

    pub fn elapsed_seconds(&self) -> f64 {
        TD(self.elapsed()).seconds()
    }

    pub fn duration_since(&self, other: &Instant) -> TimeDelta {
        self.delta - other.delta
    }

    #[allow(dead_code)]
    fn checked_add(&self, other: &Instant) -> Option<Instant> {
        Some(Instant {
            delta: self.delta.checked_add(&other.delta)?,
        })
    }

    #[allow(dead_code)]
    fn checked_sub(&self, other: &Instant) -> Option<Instant> {
        Some(Instant {
            delta: self.delta.checked_sub(&other.delta)?,
        })
    }

    #[allow(dead_code)]
    fn checked_duration_since(&self, other: &Instant) -> Option<TimeDelta> {
        self.delta.checked_sub(&other.delta)
    }
}

impl Add<TimeDelta> for Instant {
    type Output = Instant;

    fn add(self, rhs: TimeDelta) -> Self::Output {
        Instant {
            delta: self.delta + rhs,
        }
    }
}

impl AddAssign<TimeDelta> for Instant {
    fn add_assign(&mut self, rhs: TimeDelta) {
        self.delta += rhs;
    }
}

impl Sub<TimeDelta> for Instant {
    type Output = Instant;
    fn sub(self, rhs: TimeDelta) -> Self::Output {
        Instant {
            delta: self.delta - rhs,
        }
    }
}

impl SubAssign<TimeDelta> for Instant {
    fn sub_assign(&mut self, rhs: TimeDelta) {
        self.delta -= rhs
    }
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Ord, Eq, Hash)]
pub struct SystemClock {
    utc: DateTime<Utc>,
}

impl SystemClock {
    pub fn checked_add(&self, rhs: TimeDelta) -> Option<SystemClock> {
        Some(SystemClock {
            utc: self.utc.checked_add_signed(rhs)?,
        })
    }

    pub fn checked_sub(&self, rhs: TimeDelta) -> Option<SystemClock> {
        Some(SystemClock {
            utc: self.utc.checked_sub_signed(rhs)?,
        })
    }

    pub fn duration_since(&self, other: SystemClock) -> TimeDelta {
        self.utc - other.utc
    }

    pub fn elapsed(&self) -> TimeDelta {
        self.utc - DateTime::<Utc>::UNIX_EPOCH
    }
}

impl Add<TimeDelta> for SystemClock {
    type Output = SystemClock;

    fn add(self, rhs: TimeDelta) -> Self::Output {
        SystemClock {
            utc: self.utc + rhs,
        }
    }
}

impl AddAssign<TimeDelta> for SystemClock {
    fn add_assign(&mut self, rhs: TimeDelta) {
        self.utc += rhs;
    }
}

impl Sub<TimeDelta> for SystemClock {
    type Output = SystemClock;
    fn sub(self, rhs: TimeDelta) -> Self::Output {
        SystemClock {
            utc: self.utc - rhs,
        }
    }
}

impl SubAssign<TimeDelta> for SystemClock {
    fn sub_assign(&mut self, rhs: TimeDelta) {
        self.utc -= rhs
    }
}

#[derive(Debug, Clone)]
pub struct WallClock;

impl Clock for WallClock {
    fn realtime(&self) -> SystemClock {
        SystemClock { utc: Utc::now() }
    }

    fn monotonic(&self) -> Instant {
        Instant {
            delta: TimeDelta::from_std(std::time::Instant::now().elapsed()).unwrap(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct SimulatedClock {
    utc_epoch: DateTime<Utc>,
    elapsed: TimeDelta,
}

impl SimulatedClock {
    pub fn new(utc_epoch: DateTime<Utc>, elapsed: TimeDelta) -> SimulatedClock {
        SimulatedClock { utc_epoch, elapsed }
    }

    pub fn step(&mut self, delta: TimeDelta) {
        self.elapsed += delta
    }
}

impl Clock for SimulatedClock {
    fn realtime(&self) -> SystemClock {
        SystemClock {
            utc: self.utc_epoch + self.elapsed,
        }
    }

    fn monotonic(&self) -> Instant {
        Instant {
            delta: self.elapsed,
        }
    }
}

pub struct TD(pub TimeDelta);

impl TD {
    pub fn seconds(&self) -> f64 {
        self.0.num_seconds() as f64 + (self.0.subsec_nanos() as f64) / 1000000000.0
    }
}

pub fn nsec_to_sec_f64(nsec: i64) -> f64 {
    let td = TimeDelta::nanoseconds(nsec);

    td.num_seconds() as f64 + (td.subsec_nanos() as f64) / 1000000000.0
}
