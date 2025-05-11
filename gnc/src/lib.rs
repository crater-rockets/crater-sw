#![no_std]

pub mod common;
pub mod component;
pub mod component_loop;
pub mod components;
pub mod datatypes;
pub mod events;
pub mod gnc_main;
pub mod hal;

// #![cfg_attr(not(feature = "std"), no_std)]
// include generate definitions
include!(concat!(env!("OUT_DIR"), "/mod.rs"));

pub use mavlink_core::*;
use uom::si::{i64::Time, time::microsecond};

extern crate alloc;

pub type InstantU64 = fugit::Instant<u64, 1, 1_000_000>;

#[derive(Debug, Clone, Copy)]
pub struct Instant(pub InstantU64);

pub type DurationU64 = fugit::Duration<u64, 1, 1_000_000>;

#[derive(Debug, Clone, Copy)]
pub struct Duration(pub DurationU64);

impl Into<Time> for Instant {
    fn into(self) -> Time {
        Time::new::<microsecond>(self.0.duration_since_epoch().to_micros() as i64)
    }
}

impl From<InstantU64> for Instant {
    fn from(value: InstantU64) -> Self {
        Instant(value)
    }
}

impl Into<Time> for Duration {
    fn into(self) -> Time {
        Time::new::<microsecond>(self.0.to_micros() as i64)
    }
}

impl From<DurationU64> for Duration {
    fn from(value: DurationU64) -> Self {
        Duration(value)
    }
}
