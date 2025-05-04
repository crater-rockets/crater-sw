#![no_std]
#![no_main]

pub mod device;
pub mod sensors;

use embedded_alloc::TlsfHeap as Heap;

#[global_allocator]
pub static HEAP: Heap = Heap::empty();
