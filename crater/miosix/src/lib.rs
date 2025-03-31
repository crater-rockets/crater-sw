#![no_std]

extern crate alloc;

pub mod fmt;

pub mod newlib;
pub mod sync;

#[cfg(target_arch = "arm")]
pub struct LinkTest(pub f32);

use core::alloc::GlobalAlloc;
use core::ffi::{c_uint, c_void};
use core::panic::PanicInfo;

struct MiosixAllocator;

unsafe impl GlobalAlloc for MiosixAllocator {
    unsafe fn alloc(&self, layout: core::alloc::Layout) -> *mut u8 {
        unsafe { newlib::memalign(layout.align() as c_uint, layout.size() as c_uint) as *mut u8 }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, _layout: core::alloc::Layout) {
        unsafe { newlib::free(ptr as *mut c_void) }
    }

    unsafe fn realloc(
        &self,
        ptr: *mut u8,
        _layout: core::alloc::Layout,
        new_size: usize,
    ) -> *mut u8 {
        unsafe { newlib::realloc(ptr as *mut c_void, new_size as c_uint) as *mut u8 }
    }
}

#[global_allocator]
static ALLOCATOR: MiosixAllocator = MiosixAllocator;

unsafe impl Sync for MiosixAllocator {}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {} // Halt the program indefinitely
}
