#![no_std]

#[cfg(feature = "miosix")]
mod miosix_deps {
    pub type LinkTest = miosix::LinkTest;
}

#[cfg(feature = "miosix")]
pub use miosix_deps::*;

#[cfg(not(feature = "miosix"))]
mod std_deps {
    pub struct MyLinkTest(pub i32);
    pub type LinkTest = MyLinkTest;
}

#[cfg(not(feature = "miosix"))]
pub use std_deps::*;
