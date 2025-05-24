use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Blocking;
use embassy_stm32::mode::Mode;
use embassy_stm32::spi::Spi;
use embassy_stm32::spi::{self, Word};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::MutexGuard;

use super::bsp::bus::SpiType;

#[derive(Debug, Clone, Default)]
pub struct SpiDeviceConfig {
    pub read_padding_byte: bool,
}

pub struct SpiDevice<SpiMode: Mode + 'static> {
    spi: &'static SpiType<SpiMode>,
    cs: Output<'static>,
    config: SpiDeviceConfig,
}

impl<SpiMode: Mode + 'static> SpiDevice<SpiMode> {
    pub fn new(
        spi: &'static SpiType<SpiMode>,
        cs: Output<'static>,
        config: SpiDeviceConfig,
    ) -> Self {
        SpiDevice { spi, cs, config }
    }

    pub async fn start_transaction<'a>(&'a mut self) -> SpiTransaction<'a, SpiMode> {
        SpiTransaction::start(self).await
    }
}

type SpiMutexGuard<'a, SpiMode> = MutexGuard<'a, ThreadModeRawMutex, Option<Spi<'static, SpiMode>>>;

pub struct SpiTransaction<'a, SpiMode: Mode + 'static> {
    device: &'a mut SpiDevice<SpiMode>,
    spi: SpiMutexGuard<'a, SpiMode>,
}

impl<'a, SpiMode: Mode> SpiTransaction<'a, SpiMode> {
    pub async fn start(device: &'a mut SpiDevice<SpiMode>) -> Self {
        let spi = device.spi.lock().await;

        device.cs.set_low();
        SpiTransaction { device, spi }
    }

    pub fn end(self) {
        // Drop is called
    }
}

impl<'a, SpiMode: Mode> Drop for SpiTransaction<'a, SpiMode> {
    fn drop(&mut self) {
        self.device.cs.set_high();
    }
}

impl<'a> SpiTransaction<'a, Blocking> {
    fn spi(&mut self) -> &mut Spi<'static, Blocking> {
        self.spi.as_mut().unwrap()
    }

    pub fn write_raw<W: Word>(&mut self, words: &[W]) -> Result<(), spi::Error> {
        self.spi().blocking_write(words)
    }

    pub fn read_raw<W: Word>(&mut self, words: &mut [W]) -> Result<(), spi::Error> {
        self.spi().blocking_read(words)
    }

    pub fn transfer_in_place_raw<W: Word>(&mut self, words: &mut [W]) -> Result<(), spi::Error> {
        self.spi().blocking_transfer_in_place(words)
    }

    pub fn transfer_raw<W: Word>(&mut self, read: &mut [W], write: &[W]) -> Result<(), spi::Error> {
        self.spi().blocking_transfer(read, write)
    }

    pub fn read_reg_raw<W: Word>(&mut self, reg: u8, buf: &mut [W]) {
        let cmd_buf: [u8; 2] = [reg | 0x80, 0];
        
        let read_byte_index = 1 + (self.device.config.read_padding_byte as usize);
        
        let spi = self.spi();
        spi.blocking_write(&cmd_buf[0..read_byte_index]).unwrap();
        spi.blocking_read(buf).unwrap();
    }

    pub fn read_reg_u8(&mut self, reg: u8) -> u8 {
        let mut read_buf: [u8; 3] = [reg | 0x80, 0, 0];

        let read_byte_index = 1 + (self.device.config.read_padding_byte as usize);

        self.spi
            .as_mut()
            .unwrap()
            .blocking_transfer_in_place(&mut read_buf[..read_byte_index + 1])
            .unwrap();

        read_buf[read_byte_index]
    }

    pub fn write_reg_u8(&mut self, reg: u8, value: u8) {
        let mut buf: [u8; 2] = [reg & 0x7F, value];
        self.spi
            .as_mut()
            .unwrap()
            .blocking_transfer_in_place(&mut buf)
            .unwrap();
    }
}
