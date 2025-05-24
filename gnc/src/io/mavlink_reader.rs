use mavlink::{error::MessageReadError, peek_reader::PeekReader, read_v2_msg};

use crate::mav_crater;

use super::MavlinkHandler;

#[cfg(feature="std")]
use std::io::Read;

#[cfg(feature="embedded")]
use embedded_io::Read;

pub struct MavlinkReader<R, H> {
    reader: PeekReader<R>,
    handler: H,
}

impl<R, H> MavlinkReader<R, H> {
    pub fn handler(&self) -> &H {
        &self.handler
    }
}

impl<R, H> MavlinkReader<R, H>
where
    R: Read,
    H: MavlinkHandler,
{
    pub fn new(reader: R, handler: H) -> Self {
        Self {
            reader: PeekReader::new(reader),
            handler,
        }
    }

    pub fn read(&mut self) -> Result<(), MessageReadError> {
        let (header, msg) = read_v2_msg::<mav_crater::MavMessage, R>(&mut self.reader)?;
        self.handler.handle(header, msg);

        Ok(())
    }
}
