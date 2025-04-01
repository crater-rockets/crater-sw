#![no_std]

#[cfg(feature = "miosix")]
use core::panic::PanicInfo;

#[cfg(feature = "miosix")]
extern crate alloc;

#[cfg(feature = "miosix")]
mod miosix;

#[cfg(not(feature = "miosix"))]
extern crate std;

pub mod hal {
    #[cfg(feature = "miosix")]
    pub use crate::miosix::*;

    #[cfg(not(feature = "miosix"))]
    pub use std::*;
}

#[cfg(not(feature = "miosix"))]
pub use std::print as mprint;

#[cfg(not(feature = "miosix"))]
pub use std::println as mprintln;

#[cfg(feature = "miosix")]
#[global_allocator]
static ALLOCATOR: hal::alloc::MiosixAllocator = hal::alloc::MiosixAllocator;

#[cfg(feature = "miosix")]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {} // Halt the program indefinitely
}
