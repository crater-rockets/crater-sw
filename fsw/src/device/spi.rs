use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Blocking;
use embassy_stm32::mode::Mode;
use embassy_stm32::spi::Spi;
use embassy_stm32::spi::{self, Word};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::MutexGuard;

use super::bsp::SpiType;

pub struct SpiDevice<SpiMode: Mode + 'static> {
    spi: &'static SpiType<SpiMode>,
    cs: Output<'static>,
}

impl<SpiMode: Mode + 'static> SpiDevice<SpiMode> {
    pub fn new(spi: &'static SpiType<SpiMode>, cs: Output<'static>) -> Self {
        SpiDevice { spi, cs }
    }

    pub async fn start_transaction<'a>(&'a mut self) -> SpiTransaction<'a, SpiMode> {
        let lock = self.spi.lock().await;
        SpiTransaction::start(lock, &mut self.cs)
    }
}

type SpiMutexGuard<'a, SpiMode> = MutexGuard<'a, ThreadModeRawMutex, Option<Spi<'static, SpiMode>>>;

pub struct SpiTransaction<'a, SpiMode: Mode> {
    spi: SpiMutexGuard<'a, SpiMode>,
    cs: &'a mut Output<'static>,
}

impl<'a, SpiMode: Mode> SpiTransaction<'a, SpiMode> {
    pub fn start(spi: SpiMutexGuard<'a, SpiMode>, cs: &'a mut Output<'static>) -> Self {
        cs.set_low();
        SpiTransaction { spi, cs }
    }

    pub fn end(self) {
        // Drop is called
    }
}

impl<'a, SpiMode: Mode> Drop for SpiTransaction<'a, SpiMode> {
    fn drop(&mut self) {
        self.cs.set_high();
    }
}

impl<'a> SpiTransaction<'a, Blocking> {
    pub fn write_raw<W: Word>(&mut self, words: &[W]) -> Result<(), spi::Error> {
        self.spi.as_mut().unwrap().blocking_write(words)
    }

    pub fn read_raw<W: Word>(&mut self, words: &mut [W]) -> Result<(), spi::Error> {
        self.spi.as_mut().unwrap().blocking_read(words)
    }
    
    pub fn transfer_in_place_raw<W: Word>(&mut self, words: &mut [W]) -> Result<(), spi::Error> {
        self.spi.as_mut().unwrap().blocking_transfer_in_place(words)
    }

    pub fn transfer_raw<W: Word>(&mut self, read: &mut [W], write: &[W]) -> Result<(), spi::Error> {
        self.spi.as_mut().unwrap().blocking_transfer(read, write)
    }

    pub fn read_reg_raw<W: Word>(&mut self, reg: u8, buf: &mut [W]) {
        let cmd: [u8; 2] = [reg | 0x80, 0];

        let spi = self.spi.as_mut().unwrap();
        spi.blocking_write(&cmd).unwrap();
        spi.blocking_read(buf).unwrap();
    }

    pub fn read_reg_u8(&mut self, reg: u8) -> u8 {
        let mut buf: [u8; 3] = [reg | 0x80, 0, 0];
        self.spi
            .as_mut()
            .unwrap()
            .blocking_transfer_in_place(&mut buf)
            .unwrap();

        buf[2]
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
