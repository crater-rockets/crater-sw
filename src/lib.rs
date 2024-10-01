// Include the `items` module, which is generated from items.proto.
// It is important to maintain the same structure as in the proto.

use prost_reflect::DescriptorPool;
use once_cell::sync::Lazy;

pub mod quacopter {
    pub mod sensors {
        include!(concat!(env!("OUT_DIR"), "/quadcopter.sensors.rs"));
    }
}

pub mod telemetry;
pub mod utils;


static DESCRIPTOR_POOL: Lazy<DescriptorPool> = Lazy::new(|| {
    DescriptorPool::decode(
        include_bytes!(concat!(env!("OUT_DIR"), "/file_descriptor_set.bin")).as_ref(),
    )
    .unwrap()
});

