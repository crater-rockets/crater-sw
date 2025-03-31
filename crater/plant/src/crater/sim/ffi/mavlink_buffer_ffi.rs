use std::{io::Write, marker::PhantomData, os::linux::raw, slice, vec};

use bindings::crt_ffi_Buffer;
use chrono::TimeDelta;
use egui::ahash::HashMap;
use nalgebra::Vector4;

use crate::{
    core::time::{Clock, Timestamp},
    crater::sim::{gnc::ServoPosition, sensors::IMUSample},
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryDispatcher, TelemetryError, TelemetryReceiver, TelemetrySender},
    utils::capacity::Capacity::Unbounded,
};
use anyhow::Result;

use super::mav_crater::{self, crater::MavMessage};
use mavlink::{self, peek_reader::PeekReader, MessageData};

mod bindings {
    include!(concat!(env!("OUT_DIR"), "/crater_ffi.rs"));
}

#[derive(Debug, Hash, PartialEq, Eq)]
struct ChannelFromMavlinkKey {
    msg_id: u32,
    sys_id: u8,
    comp_id: u8,
}

struct ChannelToMavlinkData {
    converter: Box<dyn ToMavlinkConverter + Send>,
    system_id: u8,
    comp_id: u8,
}

pub struct MavlinkBufferFFI {
    ffi: bindings::crt_ffi_CraterCpp,
    channel_to_mavlink: Vec<ChannelToMavlinkData>,
    channel_from_mavlink: HashMap<ChannelFromMavlinkKey, Vec<Box<dyn FromMavlinkConverter + Send>>>,

    buf_to_cpp: Vec<u8>,
    seq_cnt: u8,
}

impl MavlinkBufferFFI {
    pub fn new(ctx: NodeContext) -> Result<Self> {
        let ffi = unsafe { bindings::crt_ffi_CraterCpp::new() };

        let out = MavlinkBufferFFI {
            ffi,
            channel_to_mavlink: vec![],
            channel_from_mavlink: HashMap::default(),
            buf_to_cpp: vec![],
            seq_cnt: 0u8,
        };

        Ok(out)
    }
}

impl Drop for MavlinkBufferFFI {
    fn drop(&mut self) {
        unsafe {
            self.ffi.destruct();
        }
    }
}

impl MavlinkBufferFFI {
    pub fn channel_to_buffer<T>(&mut self, system_id: u8, comp_id: u8, rx: TelemetryReceiver<T>)
    where
        ChannelToMavlink<T>: ToMavlinkConverter,
        T: Send + 'static,
    {
        self.channel_to_mavlink.push(ChannelToMavlinkData {
            converter: Box::new(ChannelToMavlink::new(rx)),
            comp_id,
            system_id,
        });
    }

    pub fn channel_from_buffer<M, T>(&mut self, sys_id: u8, comp_id: u8, tx: TelemetrySender<T>)
    where
        M: mavlink::MessageData,
        ChannelFromMavlink<T>: FromMavlinkConverter,
        T: Send + 'static,
    {
        let key = ChannelFromMavlinkKey {
            msg_id: M::ID,
            sys_id: sys_id,
            comp_id: comp_id,
        };

        if self.channel_from_mavlink.contains_key(&key) {
            self.channel_from_mavlink
                .get_mut(&key)
                .unwrap()
                .push(Box::new(ChannelFromMavlink::new(tx)));
        }
    }

    pub fn step(&mut self) -> Result<StepResult> {
        for to_mav in self.channel_to_mavlink.iter() {
            while let Ok(msg) = to_mav.converter.next_message() {
                let header = mavlink::MavHeader {
                    system_id: to_mav.system_id,
                    component_id: to_mav.comp_id,
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
            if let Ok(msg) = mav_crater::crater::ServoTarget_DATA::deser(
                mavlink::MavlinkVersion::V2,
                rawmsg.payload(),
            ) {
                println!("{:#?}", msg);
            }
        }

        self.buf_to_cpp.clear();

        Ok(StepResult::Continue)
    }
}

pub trait ToMavlinkConverter {
    fn next_message(&self) -> Result<MavMessage, TelemetryError>;
}

pub struct ChannelToMavlink<T> {
    rx: TelemetryReceiver<T>,
}

impl<T> ChannelToMavlink<T> {
    pub fn new(rx: TelemetryReceiver<T>) -> Self {
        ChannelToMavlink { rx }
    }
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

pub trait FromMavlinkConverter {
    fn parse_message(
        &self,
        raw_mavlink_msg: &mavlink::MAVLinkV2MessageRaw,
        timestamp: Timestamp,
    ) -> Result<()>;
}

pub struct ChannelFromMavlink<T> {
    tx: TelemetrySender<T>,
}

impl<T> ChannelFromMavlink<T> {
    pub fn new(tx: TelemetrySender<T>) -> Self {
        ChannelFromMavlink { tx }
    }
}

impl FromMavlinkConverter for ChannelFromMavlink<ServoPosition> {
    fn parse_message(
        &self,
        raw_msg: &mavlink::MAVLinkV2MessageRaw,
        timestamp: Timestamp,
    ) -> Result<()> {
        let mav_msg = mav_crater::crater::ServoTarget_DATA::deser(
            mavlink::MavlinkVersion::V2,
            raw_msg.payload(),
        )?;

        let pos = ServoPosition(Vector4::new(
            (mav_msg.s1_target_deg as f64).to_radians(),
            (mav_msg.s2_target_deg as f64).to_radians(),
            (mav_msg.s3_target_deg as f64).to_radians(),
            (mav_msg.s4_target_deg as f64).to_radians(),
        ));

        self.tx.send(timestamp, pos);

        Ok(())
    }
}
