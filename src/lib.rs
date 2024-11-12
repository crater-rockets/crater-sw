// Include the `items` module, which is generated from items.proto.
// It is important to maintain the same structure as in the proto.

use once_cell::sync::Lazy;
use prost_reflect::DescriptorPool;

pub mod crater;
pub mod math;
pub mod nodes;
pub mod parameters;
pub mod plot;
pub mod telemetry;
pub mod utils;
pub mod core;

pub mod crater_messages {
    pub mod basic {
        use nalgebra::{Matrix, RawStorage, Vector3, U1, U3, U4};
        use num_traits::AsPrimitive;

        include!(concat!(env!("OUT_DIR"), "/crater.basic.rs"));

        impl<T: AsPrimitive<f32>, S: RawStorage<T, U3, U1>> From<Matrix<T, U3, U1, S>> for Vec3 {
            fn from(v: Matrix<T, U3, U1, S>) -> Self {
                Vec3 {
                    x: v[0].as_(),
                    y: v[1].as_(),
                    z: v[2].as_(),
                }
            }
        }

        impl<T: AsPrimitive<f32>, S: RawStorage<T, U4, U1>> From<Matrix<T, U4, U1, S>> for Quaternion {
            fn from(v: Matrix<T, U4, U1, S>) -> Self {
                Quaternion {
                    x: v[0].as_(),
                    y: v[1].as_(),
                    z: v[2].as_(),
                    w: v[3].as_(),
                }
            }
        }

        impl<T: From<f32>> From<Vec3> for Vector3<T> {
            fn from(v: Vec3) -> Self {
                Vector3::new(v.x.into(), v.y.into(), v.z.into())
            }
        }

        impl<T: From<f32>> From<Quaternion> for nalgebra::Quaternion<T> {
            fn from(v: Quaternion) -> Self {
                nalgebra::Quaternion::new(v.w.into(), v.x.into(), v.y.into(), v.z.into())
            }
        }
    }

    pub mod sensors {
        include!(concat!(env!("OUT_DIR"), "/crater.sensors.rs"));
    }
}

pub static DESCRIPTOR_POOL: Lazy<DescriptorPool> = Lazy::new(|| {
    DescriptorPool::decode(
        include_bytes!(concat!(env!("OUT_DIR"), "/file_descriptor_set.bin")).as_ref(),
    )
    .unwrap()
});
