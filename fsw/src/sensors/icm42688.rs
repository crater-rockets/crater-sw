use embassy_stm32::mode::Blocking;
use thiserror::Error;

use crate::device::spi::SpiDevice;

const CHIP_ID: u8 = 0x47;

pub mod regs {
    pub enum Addr {
        WhoAmI = 0x75,
    }
}

#[derive(Debug, Error)]
pub enum Error {
    #[error("Bad Chip ID: {0}. Expected 71")]
    BadChipIp(u8),
}
/// 688 ONLYU
/// - AA filter configuration: p.28
/// - User programmable offset p.30
///
///
///
///
pub struct Icm42688 {
    spi_dev: SpiDevice<Blocking>,
}

impl Icm42688 {
    pub async fn init(mut spi_dev: SpiDevice<Blocking>) -> Result<Self, Error> {
        let mut remaining_attempts = 3;

        while remaining_attempts >= 0 {
            let chip_id = spi_dev
                .start_transaction()
                .await
                .read_reg_u8(regs::Addr::WhoAmI as u8);

            if chip_id == CHIP_ID {
                break;
            } else if remaining_attempts == 0 {
                return Err(Error::BadChipIp(chip_id));
            } else {
                remaining_attempts -= 1;
            }
        }
        
        Ok(Self { spi_dev })
    }
}
