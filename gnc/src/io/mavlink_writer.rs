use alloc::{boxed::Box, vec::Vec};
use mavlink::{MavHeader, write_v2_msg};

#[cfg(feature = "embedded")]
use mavlink::write_v2_msg_async;

use crate::{hal::channel::Receiver, mav_crater};

#[cfg(feature = "std")]
use std::io::Write;

#[cfg(feature = "embedded")]
use embedded_io::Write;

pub struct MavlinkWriter<W> {
    writer: W,
    channels: Vec<Box<dyn Receiver<mav_crater::MavMessage>>>,
    seq_cnt: u8,
    err_cnt: usize,
}

impl<W> MavlinkWriter<W> {
    fn new_impl(writer: W, channels: Vec<Box<dyn Receiver<mav_crater::MavMessage>>>) -> Self {
        Self {
            writer,
            channels,
            seq_cnt: 0,
            err_cnt: 0,
        }
    }

    pub fn error_count(&self) -> usize {
        self.err_cnt
    }
    // fn iter_messages<'a>(&'a mut self) -> MavlinkWriterMessageIterator<'a, W> {
    //     MavlinkWriterMessageIterator::new(self)
    // }
}

// struct MavlinkWriterMessageIterator<'a, W> {
//     writer: &'a mut MavlinkWriter<W>,
//     curr_channel: usize,
// }

// impl<'a, W> MavlinkWriterMessageIterator<'a, W> {
//     fn new(writer: &'a mut MavlinkWriter<W>) -> Self {
//         Self {
//             writer,
//             curr_channel: 0,
//         }
//     }
// }

// impl<'a, W> Iterator for MavlinkWriterMessageIterator<'a, W> {
//     type Item = Timestamped<mav_crater::MavMessage>;

//     fn next(&mut self) -> Option<Self::Item> {
//         if let Some(msg) = self.writer.channels[self.curr_channel].try_recv() {
//             Some(msg)
//         } else {
//             self.curr_channel += 1;

//             if self.curr_channel < self.writer.channels.len() {
//                 self.next()
//             } else {
//                 None
//             }
//         }
//     }
// }

impl<W: Write> MavlinkWriter<W> {
    pub fn new(writer: W, channels: Vec<Box<dyn Receiver<mav_crater::MavMessage>>>) -> Self {
        Self::new_impl(writer, channels)
    }

    pub fn write(&mut self) {
        for receiver in self.channels.iter_mut() {
            while let Some(msg) = receiver.try_recv() {
                let header = MavHeader {
                    component_id: 0,
                    system_id: 0,
                    sequence: self.seq_cnt,
                };

                match write_v2_msg(&mut self.writer, header, &msg.v) {
                    Ok(_) => self.seq_cnt = self.seq_cnt.wrapping_add(1),
                    Err(_) => self.err_cnt = self.err_cnt.wrapping_add(1),
                }
            }
        }
    }
}

#[cfg(feature = "embedded")]
impl<W: embedded_io_async::Write> MavlinkWriter<W> {
    pub fn new_async(writer: W, channels: Vec<Box<dyn Receiver<mav_crater::MavMessage>>>) -> Self {
        Self::new_impl(writer, channels)
    }

    #[allow(unused)]
    async fn write_async(&mut self) {
        for receiver in self.channels.iter_mut() {
            while let Some(msg) = receiver.try_recv() {
                let header = MavHeader {
                    component_id: 0,
                    system_id: 0,
                    sequence: self.seq_cnt,
                };

                match write_v2_msg_async(&mut self.writer, header, &msg.v).await {
                    Ok(_) => self.seq_cnt = self.seq_cnt.wrapping_add(1),
                    Err(_) => self.err_cnt = self.err_cnt.wrapping_add(1),
                }
            }
        }
    }
}
