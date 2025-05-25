use core::array;

use crater_gnc::{Duration, common::Ts, datatypes::sensors::ImuSensorSample};
use defmt::{info, warn};
use embassy_stm32::mode::Blocking;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Instant, Timer};
use regs::{AccelMode, Addr, GyroMode};
use thiserror::Error;
use uom::si::{
    acceleration::meter_per_second_squared,
    angular_velocity::degree_per_second,
    f32::{Acceleration, AngularVelocity, ThermodynamicTemperature},
    thermodynamic_temperature::degree_celsius,
};

use crate::device::spi::SpiDevice;

const CHIP_ID: u8 = 0x47;

pub struct Config {
    pub gyro_odr: regs::GyroDataRate,
    pub gyro_fs: regs::GyroFullScale,

    pub accel_odr: regs::AccelDataRate,
    pub accel_fs: regs::AccelFullScale,
}

pub mod regs {
    use bitbybit::{bitenum, bitfield};

    pub enum Addr {
        TempData1 = 0x1D,

        IntStatus = 0x2D,

        PwrMgmt0 = 0x4E,
        GyroConfig0 = 0x4F,
        AccelConfig0 = 0x50,

        IntSource0 = 0x65,

        WhoAmI = 0x75,
        RegBankSel = 0x76,
    }

    #[bitenum(u2, exhaustive = true)]
    #[derive(Debug)]
    pub enum AccelMode {
        Off1 = 0b00,
        Off2 = 0b01,
        LowPower = 0b10,
        LowNoise = 0b11,
    }

    #[bitenum(u2, exhaustive = true)]
    #[derive(Debug)]
    pub enum GyroMode {
        Off = 0b00,
        StandBy = 0b01,
        Reserver = 0b10,
        LowNoise = 0b11,
    }

    #[bitfield(u8)]
    #[derive(Debug)]
    pub(super) struct PwrMgmt0 {
        #[bits(0..=1, rw)]
        pub accel_mode: AccelMode,

        #[bits(2..=3, rw)]
        pub gyro_mode: GyroMode,

        #[bit(4, rw)]
        pub idle: bool,

        #[bit(5, rw)]
        pub temp_dis: bool,
    }

    #[bitenum(u3, exhaustive = true)]
    pub enum GyroFullScale {
        Fs2000dps = 0b000,
        Fs1000dps = 0b001,
        Fs500dps = 0b010,
        Fs250dps = 0b011,
        Fs125dps = 0b100,
        Fs62_5dps = 0b101,
        Fs31_25dps = 0b110,
        Fs15_625dps = 0b111,
    }

    #[bitenum(u4, exhaustive = false)]
    pub enum GyroDataRate {
        Odr32khz = 0b0001,
        Odr16khz = 0b0010,
        Odr8khz = 0b0011,
        Odr4khz = 0b0100,
        Odr2khz = 0b0101,
        Odr1khz = 0b0110,
        Odr200hz = 0b0111,
        Odr100hz = 0b1000,
        Odr50hz = 0b1001,
        Odr25hz = 0b1010,
        Odr12_5hz = 0b1011,
        Odr500hz = 0b1111,
    }

    #[bitfield(u8)]
    #[derive(Debug)]
    pub(super) struct GyroConfig0 {
        #[bits(0..=3, rw)]
        pub odr: Option<GyroDataRate>,

        #[bits(5..=7, rw)]
        pub fs: GyroFullScale,
    }

    #[bitenum(u3, exhaustive = false)]
    pub enum AccelFullScale {
        Fs16g = 0b000,
        Fs8g = 0b001,
        Fs4g = 0b010,
        Fs2g = 0b011,
    }

    #[bitenum(u4, exhaustive = false)]
    pub enum AccelDataRate {
        Odr32khz = 0b0001,
        Odr16khz = 0b0010,
        Odr8khz = 0b0011,
        Odr4khz = 0b0100,
        Odr2khz = 0b0101,
        Odr1khz = 0b0110,
        Odr200hz = 0b0111,
        Odr100hz = 0b1000,
        Odr50hz = 0b1001,
        Odr25hz = 0b1010,
        Odr12_5hz = 0b1011,
        Odr6_25hz = 0b1100,
        Odr3_125hz = 0b1101,
        Odr1_5625hz = 0b1110,
        Odr500hz = 0b1111,
    }

    #[bitfield(u8)]
    #[derive(Debug)]
    pub(super) struct AccelConfig0 {
        #[bits(0..=3, rw)]
        pub odr: Option<AccelDataRate>,

        #[bits(5..=7, rw)]
        pub fs: Option<AccelFullScale>,
    }

    #[bitfield(u8)]
    #[derive(Debug)]
    pub(super) struct IntSource0 {
        #[bit(0, rw)]
        pub ui_agc_rdy_int1_en: bool,

        #[bit(1, rw)]
        pub fifo_full_int1_en: bool,

        #[bit(2, rw)]
        pub ui_drdy_fifo_ths_int1_en: bool,

        #[bit(3, rw)]
        pub ui_drdy_int1_en: bool,

        #[bit(4, rw)]
        pub reset_done_int1_en: bool,

        #[bit(5, rw)]
        pub pll_rdy_int1_en: bool,

        #[bit(6, rw)]
        pub ui_fsync_int1_en1: bool,
    }

    #[bitenum(u3, exhaustive = false)]
    pub(super) enum Bank {
        Bank0 = 0b000,
        Bank1 = 0b001,
        Bank2 = 0b010,
        Bank3 = 0b011,
        Bank4 = 0b100,
    }
    #[bitfield(u8)]
    #[derive(Debug)]
    pub(super) struct RegBankSel {
        #[bits(0..=2, rw)]
        pub bank_sel: Option<Bank>,
    }

    #[bitfield(u8)]
    #[derive(Debug)]
    pub(super) struct IntStatus {
        #[bit(0, rw)]
        pub agc_rdy_int: bool,

        #[bit(1, rw)]
        pub fifo_full_int: bool,

        #[bit(2, rw)]
        pub fifo_ths_int: bool,

        #[bit(3, rw)]
        pub drdy_int: bool,

        #[bit(4, rw)]
        pub reset_done_int: bool,

        #[bit(5, rw)]
        pub pll_rdy_int: bool,

        #[bit(6, rw)]
        pub ui_fsync_int: bool,
    }
}

#[derive(Debug, Error)]
pub enum Error {
    #[error("Bad Chip ID: {0}. Expected 71")]
    BadChipIp(u8),
}

#[derive(Debug, Clone)]
pub struct Icm42688Sample {
    pub data: ImuSensorSample,
}

pub struct Icm42688 {
    spi_dev: SpiDevice<Blocking>,
    config: Config,

    sig_drdy_timestamp: &'static Signal<CriticalSectionRawMutex, (Instant, u8)>,
}

impl Icm42688 {
    pub async fn init(
        mut spi_dev: SpiDevice<Blocking>,
        config: Config,
        sig_drdy_timestamp: &'static Signal<CriticalSectionRawMutex, (Instant, u8)>,
    ) -> Result<Self, Error> {
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

        // Set gyro ODR & FS
        let gyro_config0 = regs::GyroConfig0::new_with_raw_value(0)
            .with_odr(config.gyro_odr)
            .with_fs(config.gyro_fs);
        spi_dev
            .start_transaction()
            .await
            .write_reg_u8(Addr::GyroConfig0 as u8, gyro_config0.raw_value());

        // Set accel ODR & FS
        let accel_config0 = regs::AccelConfig0::new_with_raw_value(0)
            .with_odr(config.accel_odr)
            .with_fs(config.accel_fs);
        spi_dev
            .start_transaction()
            .await
            .write_reg_u8(Addr::AccelConfig0 as u8, accel_config0.raw_value());

        // Setup interrupts
        let int_source0 = regs::IntSource0::new_with_raw_value(0).with_ui_drdy_int1_en(true);
        spi_dev
            .start_transaction()
            .await
            .write_reg_u8(Addr::IntSource0 as u8, int_source0.raw_value());

        // Enable accel & gyro
        let pwr_mgmt0 = regs::PwrMgmt0::new_with_raw_value(0)
            .with_accel_mode(AccelMode::LowNoise)
            .with_gyro_mode(GyroMode::LowNoise);

        spi_dev
            .start_transaction()
            .await
            .write_reg_u8(Addr::PwrMgmt0 as u8, pwr_mgmt0.raw_value());

        // Wait at least 200 us for gyro & accel to start
        Timer::after_micros(200).await;

        // Clear interrupt status:
        let _ = spi_dev
            .start_transaction()
            .await
            .read_reg_u8(regs::Addr::IntStatus as u8);

        Ok(Self {
            spi_dev,
            config,
            sig_drdy_timestamp,
        })
    }

    async fn select_bank(spi_dev: &mut SpiDevice<Blocking>, bank: regs::Bank) {
        spi_dev.start_transaction().await.write_reg_u8(
            Addr::PwrMgmt0 as u8,
            regs::RegBankSel::new_with_raw_value(0)
                .with_bank_sel(bank)
                .raw_value(),
        );
    }

    pub async fn sample(&mut self) -> Ts<Icm42688Sample> {
        let (drdy_ts, overrun_count) = self.sig_drdy_timestamp.wait().await;
        let latency = Instant::now() - drdy_ts;
        self.sig_drdy_timestamp.reset();

        let mut buf = [0u8; 14];

        self.spi_dev
            .start_transaction()
            .await
            .read_reg_raw(regs::Addr::TempData1 as u8, &mut buf);

        // info!("Int status: {:#?}", reg.drdy_int());

        // if !reg.drdy_int() {
        //     warn!("ICM42688 | Sampled data but no interrupt");
        // }

        let raw_temp = i16::from_be_bytes(buf[0..2].try_into().unwrap());

        let raw_accel = [
            i16::from_be_bytes(buf[2..4].try_into().unwrap()),
            i16::from_be_bytes(buf[4..6].try_into().unwrap()),
            i16::from_be_bytes(buf[6..8].try_into().unwrap()),
        ];

        let raw_angvel = [
            i16::from_be_bytes(buf[8..10].try_into().unwrap()),
            i16::from_be_bytes(buf[10..12].try_into().unwrap()),
            i16::from_be_bytes(buf[12..14].try_into().unwrap()),
        ];

        Ts::from_microseconds(
            drdy_ts.as_micros(),
            Icm42688Sample {
                data: ImuSensorSample {
                    accel: self.convert_accel(&raw_accel),
                    ang_vel: self.convert_gyro(&raw_angvel),
                    temperature: Some(self.convert_temperature(raw_temp)),
                    int_latency: crater_gnc::DurationU64::micros(latency.as_micros()).into(),
                    overrun_count,
                },
            },
        )
    }

    fn convert_accel(&self, raw_accel: &[i16; 3]) -> [Acceleration; 3] {
        let g = 9.80665f32;

        let scale = match self.config.accel_fs {
            regs::AccelFullScale::Fs16g => 2048f32,
            regs::AccelFullScale::Fs8g => 4096f32,
            regs::AccelFullScale::Fs4g => 8192f32,
            regs::AccelFullScale::Fs2g => 16384f32,
        };

        array::from_fn(|i| {
            Acceleration::new::<meter_per_second_squared>(raw_accel[i] as f32 / scale * g)
        })
    }

    fn convert_gyro(&self, raw_angvel: &[i16; 3]) -> [AngularVelocity; 3] {
        let scale = match self.config.gyro_fs {
            regs::GyroFullScale::Fs2000dps => 16.4f32,
            regs::GyroFullScale::Fs1000dps => 32.8f32,
            regs::GyroFullScale::Fs500dps => 65.5f32,
            regs::GyroFullScale::Fs250dps => 131f32,
            regs::GyroFullScale::Fs125dps => 262f32,
            regs::GyroFullScale::Fs62_5dps => 524.3f32,
            regs::GyroFullScale::Fs31_25dps => 1048.6f32,
            regs::GyroFullScale::Fs15_625dps => 2097.2f32,
        };

        array::from_fn(|i| AngularVelocity::new::<degree_per_second>(raw_angvel[i] as f32 / scale))
    }

    fn convert_temperature(&self, raw_temp: i16) -> ThermodynamicTemperature {
        ThermodynamicTemperature::new::<degree_celsius>(raw_temp as f32 / 13.48f32 + 25.0f32)
    }
}
