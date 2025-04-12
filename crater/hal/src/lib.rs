#![no_std]

#[cfg(target_os = "none")]
use core::panic::PanicInfo;

#[cfg(target_os = "none")]
extern crate alloc;

#[cfg(target_os = "none")]
mod miosix;

#[cfg(not(target_os = "none"))]
extern crate std;

pub mod hal {
    #[cfg(target_os = "none")]
    pub use crate::miosix::std::*;

    #[cfg(not(target_os = "none"))]
    pub use std::*;
}

#[cfg(not(target_os = "none"))]
pub use std::print as mprint;

#[cfg(not(target_os = "none"))]
pub use std::println as mprintln;

#[cfg(target_os = "none")]
#[global_allocator]
static ALLOCATOR: hal::alloc::MiosixAllocator = hal::alloc::MiosixAllocator;

#[cfg(target_os = "none")]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {} // Halt the program indefinitely
}
