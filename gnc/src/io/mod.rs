use mavlink::MavHeader;

use crate::mav_crater;

pub mod mavlink_dispatcher;
pub mod mavlink_reader;
pub mod mavlink_writer;

pub const MAVLINK_MSG_MAX_SIZE: usize = 280;

pub trait MavlinkHandler {
    fn handle(&mut self, header: MavHeader, msg: mav_crater::MavMessage);
}

#[cfg(test)]
mod tests {
    use alloc::vec::Vec;
    use mavlink::{MavHeader, write_v2_msg};

    use crate::mav_crater::{self, MavMessage, SensBmp390_DATA};

    use super::{MavlinkHandler, mavlink_reader::MavlinkReader, mavlink_writer::MavlinkWriter};

    #[derive(Debug, Default)]
    struct TestHandler {
        press_cnt: usize,
    }

    impl MavlinkHandler for TestHandler {
        fn handle(&mut self, header: mavlink::MavHeader, msg: mav_crater::MavMessage) {
            match msg {
                MavMessage::SensBmp390(data) => {
                    self.press_cnt += 1;
                }
                _ => {}
            }
        }
    }

    #[test]
    fn test_mavlink_reader() {
        let mut buf: Vec<u8> = Vec::new();
        buf.resize(280 * 5, 0);

        let header = MavHeader {
            component_id: 0,
            system_id: 0,
            sequence: 0,
        };

        write_v2_msg(
            &mut buf,
            header,
            &MavMessage::SensBmp390(SensBmp390_DATA::DEFAULT),
        )
        .unwrap();
        write_v2_msg(
            &mut buf,
            header,
            &MavMessage::SensBmp390(SensBmp390_DATA::DEFAULT),
        )
        .unwrap();

        let mut handler = TestHandler::default();
        let mut reader = MavlinkReader::new(buf.as_slice(), handler);

        assert!(reader.read().is_ok());
        assert_eq!(reader.handler().press_cnt, 1);

        assert!(reader.read().is_ok());
        assert_eq!(reader.handler().press_cnt, 2);

        assert!(!reader.read().is_ok());
    }
}
