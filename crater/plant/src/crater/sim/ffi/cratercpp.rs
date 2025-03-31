use std::{io::Write, marker::PhantomData, os::linux::raw, slice, vec};

use bindings::crt_ffi_Buffer;
use chrono::TimeDelta;

use crate::{
    core::time::Clock,
    crater::sim::sensors::IMUSample,
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryDispatcher, TelemetryError, TelemetryReceiver},
    utils::capacity::Capacity::Unbounded,
};
use anyhow::Result;

use super::mav_crater::{self, crater::MavMessage};
use mavlink::{self, peek_reader::PeekReader, MessageData};

mod bindings {
    include!(concat!(env!("OUT_DIR"), "/crater_ffi.rs"));
}

pub struct CraterCpp {
    ffi: bindings::crt_ffi_CraterCpp,
    channel_to_mavlink: Vec<Box<dyn ToMavlinkConverter + Send>>,

    buf_to_cpp: Vec<u8>,
    seq_cnt: u8,
}

impl CraterCpp {
    pub fn new(ctx: NodeContext) -> Result<Self> {
        let mut channel_to_mavlink: Vec<Box<dyn ToMavlinkConverter + Send>> = vec![];

        channel_to_mavlink.push(Box::new(ChannelToMavlink {
            rx: ctx
                .telemetry()
                .subscribe::<IMUSample>("/sensors/imu", Unbounded)?,
        }));

        let ffi = unsafe { bindings::crt_ffi_CraterCpp::new() };

        let out = CraterCpp {
            ffi,
            channel_to_mavlink,
            buf_to_cpp: vec![],
            seq_cnt: 0u8,
        };

        Ok(out)
    }
}

impl Drop for CraterCpp {
    fn drop(&mut self) {
        unsafe {
            self.ffi.destruct();
        }
    }
}

trait ToMavlinkConverter {
    fn next_message(&self) -> Result<MavMessage, TelemetryError>;
}

struct ChannelToMavlink<T> {
    rx: TelemetryReceiver<T>,
}

impl ToMavlinkConverter for ChannelToMavlink<IMUSample> {
    fn next_message(&self) -> Result<MavMessage, TelemetryError> {
        let sample = self.rx.try_recv()?;

        Ok(MavMessage::Sensor6DOFImu(
            mav_crater::crater::Sensor6DOFImu_DATA {
                timestamp_us: sample.0.monotonic.elapsed().num_microseconds().unwrap(),

                imu_id: mav_crater::crater::Imusensorid::IMU_16_G,

                acc_x_body_m_s2: sample.1.acc[0] as f32,
                acc_y_body_m_s2: sample.1.acc[1] as f32,
                acc_z_body_m_s2: sample.1.acc[1] as f32,

                gyro_x_body_rad_s: sample.1.gyro[0] as f32,
                gyro_y_body_rad_s: sample.1.gyro[1] as f32,
                gyro_z_body_rad_s: sample.1.gyro[2] as f32,
            },
        ))
    }
}

impl Node for CraterCpp {
    fn step(&mut self, i: usize, dt: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
        for converter in self.channel_to_mavlink.iter() {
            while let Ok(msg) = converter.next_message() {
                let header = mavlink::MavHeader {
                    system_id: mav_crater::SYSTEM_ID,
                    component_id: mav_crater::crater::Componentid::Rocket as u8,
                    sequence: self.seq_cnt,
                };

                let _ = mavlink::write_v2_msg(&mut self.buf_to_cpp, header, &msg)?;

                self.seq_cnt = self.seq_cnt.wrapping_add(1);
            }
        }

        let buf_to_cpp = crt_ffi_Buffer {
            data: self.buf_to_cpp.as_mut_ptr(),
            length: self.buf_to_cpp.len() as u64,
        };

        let mut buf_from_cpp = crt_ffi_Buffer {
            data: std::ptr::null_mut(),
            length: 0u64,
        };

        unsafe { self.ffi.step(buf_to_cpp, &mut buf_from_cpp) };

        let cpp_out =
            unsafe { slice::from_raw_parts(buf_from_cpp.data, buf_from_cpp.length as usize) };

        let mut reader: PeekReader<&[u8]> = mavlink::peek_reader::PeekReader::new(cpp_out.as_ref());

        while let Ok(rawmsg) = mavlink::read_v2_raw_message::<MavMessage, &[u8]>(&mut reader) {
            if let Ok(msg) = mav_crater::crater::ServoTarget_DATA::deser(mavlink::MavlinkVersion::V2, rawmsg.payload()) {
                println!("{:#?}", msg);
            }
        }

        self.buf_to_cpp.clear();

        Ok(StepResult::Continue)
    }
}
