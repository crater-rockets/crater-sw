use core::alloc::GlobalAlloc;
use core::ffi::{c_uint, c_void};

pub struct MiosixAllocator;

unsafe impl GlobalAlloc for MiosixAllocator {
    unsafe fn alloc(&self, layout: core::alloc::Layout) -> *mut u8 {
        unsafe { crate::hal::newlib::memalign(layout.align() as c_uint, layout.size() as c_uint) as *mut u8 }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, _layout: core::alloc::Layout) {
        unsafe { crate::hal::newlib::free(ptr as *mut c_void) }
    }

    unsafe fn realloc(
        &self,
        ptr: *mut u8,
        _layout: core::alloc::Layout,
        new_size: usize,
    ) -> *mut u8 {
        unsafe { crate::hal::newlib::realloc(ptr as *mut c_void, new_size as c_uint) as *mut u8 }
    }
}

unsafe impl Sync for MiosixAllocator {}
