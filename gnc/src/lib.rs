#![no_std]

pub mod channel;
pub mod component_loop;
pub mod component;
pub mod components;
pub mod events;
pub mod hal;

// #![cfg_attr(not(feature = "std"), no_std)]
// include generate definitions
include!(concat!(env!("OUT_DIR"), "/mod.rs"));

pub use mavlink_core::*;

extern crate alloc;

pub type Instant = fugit::Instant<u64, 1, 1_000_000>;
pub type Duration = fugit::Duration<u64, 1, 1_000_000>;