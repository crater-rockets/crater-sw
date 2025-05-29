use crater_gnc::{common::Ts, datatypes::sensors::PressureSensorSample};
use defmt::{debug, error};
use embassy_stm32::mode::Blocking;
use embassy_time::Instant;
use libm::{ceilf, log2f, powf};
use thiserror::{self, Error};
use uom::si::{
    f32::{Pressure, ThermodynamicTemperature},
    pressure::pascal,
    thermodynamic_temperature::degree_celsius,
};

use crate::device::spi::SpiDevice;
use {defmt_rtt as _, panic_probe as _};

const CHIP_ID: u8 = 0x60;

#[allow(unused)]
pub mod regs {
    use arbitrary_int::*;
    use bitbybit::*;

    pub enum Addr {
        ChipId = 0x00,
        RevId = 0x01,
        ErrReg = 0x02,
        Status = 0x03,
        PressData0 = 0x04,
        PressData1 = 0x05,
        PressData2 = 0x06,
        TempData0 = 0x07,
        TempData1 = 0x08,
        TempData2 = 0x09,

        SensorTime0 = 0x0C,
        SensorTime1 = 0x0D,
        SensorTime2 = 0x0E,

        Event = 0x10,
        IntStatus = 0x11,
        FifoLength0 = 0x12,
        FifoLength1 = 0x13,
        FifoData = 0x14,
        FifoWtm0 = 0x15,
        FifoWtm1 = 0x16,
        FigoConfig1 = 0x17,
        FigoConfig2 = 0x18,
        IntCtrl = 0x19,

        CompensationParams = 0x31,

        IfConf = 0x1A,
        PwrCtrl = 0x1B,
        Osr = 0x1C,
        Odr = 0x1D,
        Config = 0x1F,

        Cmd = 0x7E,
    }

    #[bitenum(u5, exhaustive = false)]
    pub enum DataRateValue {
        Odr200 = 0x00,
        Odr100 = 0x01,
        Odr50 = 0x02,
        Odr25 = 0x03,
        Odr12p5 = 0x04,
        Odr6p25 = 0x05,
        Odr3p1 = 0x06,
        Odr1p5 = 0x07,
        Odr0p78 = 0x08,
        Odr0p39 = 0x09,
        Odr0p2 = 0x0A,
        Odr0p1 = 0x0B,
        Odr0p05 = 0x0C,
        Odr0p02 = 0x0D,
        Odr0p01 = 0x0E,
        Odr0p006 = 0x0F,
        Odr0p003 = 0x10,
        Odr0p0015 = 0x11,
    }

    #[bitenum(u2, exhaustive = true)]
    pub enum PowerModeValue {
        Sleep = 0x00,
        Forced1 = 0x01,
        Forced2 = 0x02,
        Normal = 0x03,
    }

    #[bitfield(u8)]
    #[derive(Debug)]
    pub(super) struct Status {
        #[bit(4, r)]
        cmd_rdy: bool,

        #[bit(5, r)]
        drdy_press: bool,

        #[bit(6, r)]
        drdy_temp: bool,
    }

    #[bitfield(u8)]
    #[derive(Debug)]
    pub(super) struct PwrCtrl {
        #[bit(0, rw)]
        press_en: bool,

        #[bit(1, rw)]
        temp_en: bool,

        #[bits(4..=5, rw)]
        mode: PowerModeValue,
    }

    #[bitfield(u8)]
    #[derive(Debug)]
    pub(super) struct Odr {
        #[bits(0..=4, rw)]
        odr: Option<DataRateValue>,
    }

    #[bitenum(u3, exhaustive = false)]
    pub enum OversamplingValue {
        None = 0b000,
        Times2 = 0b001,
        Times4 = 0b010,
        Times8 = 0b011,
        Times16 = 0b100,
        Times32 = 0b101,
    }

    #[bitfield(u8)]
    #[derive(Debug)]
    pub(super) struct Osr {
        #[bits(0..=2, rw)]
        osr_p: Option<OversamplingValue>,

        #[bits(3..=5, rw)]
        osr_t: Option<OversamplingValue>,
    }
}

#[derive(Debug, Error)]
pub enum Error {
    #[error("Bad Chip ID: {0}. Expected 96")]
    BadChipIp(u8),

    #[error("Selected ODR not compatible with provided OSR settings")]
    BadOdr,
}

pub struct Config {
    pub odr: regs::DataRateValue,

    pub osr_p: regs::OversamplingValue,
    pub osr_t: regs::OversamplingValue,
}

struct Compensation {
    t1: f32,
    t2: f32,
    t3: f32,
    p1: f32,
    p2: f32,
    p3: f32,
    p4: f32,
    p5: f32,
    p6: f32,
    p7: f32,
    p8: f32,
    p9: f32,
    p10: f32,
    p11: f32,
}

impl Compensation {
    fn from_raw_bytes(bytes: &[u8; 21]) -> Self {
        let t1: u16 = u16::from_le_bytes(bytes[0..2].try_into().unwrap());
        let t2: u16 = u16::from_le_bytes(bytes[2..4].try_into().unwrap());
        let t3: i8 = i8::from_le_bytes(bytes[4..5].try_into().unwrap());

        let p1: i16 = i16::from_le_bytes(bytes[5..7].try_into().unwrap());
        let p2: i16 = i16::from_le_bytes(bytes[7..9].try_into().unwrap());
        let p3: i8 = i8::from_le_bytes(bytes[9..10].try_into().unwrap());
        let p4: i8 = i8::from_le_bytes(bytes[10..11].try_into().unwrap());
        let p5: u16 = u16::from_le_bytes(bytes[11..13].try_into().unwrap());
        let p6: u16 = u16::from_le_bytes(bytes[13..15].try_into().unwrap());
        let p7: i8 = i8::from_le_bytes(bytes[15..16].try_into().unwrap());
        let p8: i8 = i8::from_le_bytes(bytes[16..17].try_into().unwrap());
        let p9: i16 = i16::from_le_bytes(bytes[17..19].try_into().unwrap());
        let p10: i8 = i8::from_le_bytes(bytes[19..20].try_into().unwrap());
        let p11: i8 = i8::from_le_bytes(bytes[20..21].try_into().unwrap());

        Compensation {
            t1: t1 as f32 / powf(2f32, -8f32),
            t2: t2 as f32 / powf(2f32, 30f32),
            t3: t3 as f32 / powf(2f32, 48f32),

            p1: (p1 as f32 - powf(2f32, 14f32)) / powf(2f32, 20f32),
            p2: (p2 as f32 - powf(2f32, 14f32)) / powf(2f32, 29f32),
            p3: p3 as f32 / powf(2f32, 32f32),
            p4: p4 as f32 / powf(2f32, 37f32),
            p5: p5 as f32 / powf(2f32, -3f32),
            p6: p6 as f32 / powf(2f32, 6f32),
            p7: p7 as f32 / powf(2f32, 8f32),
            p8: p8 as f32 / powf(2f32, 15f32),
            p9: p9 as f32 / powf(2f32, 48f32),
            p10: p10 as f32 / powf(2f32, 48f32),
            p11: p11 as f32 / powf(2f32, 65f32),
        }
    }

    fn temperature(&self, raw_temp: u32) -> ThermodynamicTemperature {
        let pd1 = raw_temp as f32 - self.t1;
        let pd2 = pd1 * self.t2;

        ThermodynamicTemperature::new::<degree_celsius>(pd2 + (pd1 * pd1) * self.t3)
    }

    fn pressure(&self, raw_press: u32, temp: ThermodynamicTemperature) -> Pressure {
        let temp_degc = temp.get::<degree_celsius>();

        let raw_press = raw_press as f32;

        let pd1 = self.p6 * temp_degc;
        let pd2 = self.p7 * (temp_degc * temp_degc);
        let pd3 = self.p8 * (temp_degc * temp_degc * temp_degc);
        let po1 = self.p5 + pd1 + pd2 + pd3;

        let pd1 = self.p2 * temp_degc;
        let pd2 = self.p3 * (temp_degc * temp_degc);
        let pd3 = self.p4 * (temp_degc * temp_degc * temp_degc);
        let po2 = raw_press * (self.p1 + pd1 + pd2 + pd3);

        let pd1 = raw_press * raw_press;
        let pd2 = self.p9 + self.p10 * temp_degc;
        let pd3 = pd1 * pd2;
        let pd4 = pd3 + raw_press * raw_press * raw_press * self.p11;

        Pressure::new::<pascal>(po1 + po2 + pd4)
    }
}

pub struct Bmp390 {
    spi_dev: SpiDevice<Blocking>,
    compensation: Compensation,
}

pub struct Bmp390Sample {
    pub raw_press: u32,
    pub raw_temp: u32,

    pub value: PressureSensorSample,
}

impl Bmp390 {
    pub async fn init(mut spi_dev: SpiDevice<Blocking>, config: Config) -> Result<Self, Error> {
        if !Self::check_odr(config.odr, config.osr_p, config.osr_t) {
            return Err(Error::BadOdr);
        }

        let mut remaining_attempts = 3;

        while remaining_attempts >= 0 {
            let chip_id = spi_dev
                .start_transaction()
                .await
                .read_reg_u8(regs::Addr::ChipId as u8);

            if chip_id == CHIP_ID {
                break;
            } else if remaining_attempts == 0 {
                return Err(Error::BadChipIp(chip_id));
            } else {
                debug!("BMP390 | Bad chip id: {}. Retrying", chip_id);
                remaining_attempts -= 1;
            }
        }

        let compensation = {
            let mut buf = [0 as u8; 21];
            spi_dev
                .start_transaction()
                .await
                .read_reg_raw(regs::Addr::CompensationParams as u8, &mut buf);

            Compensation::from_raw_bytes(&buf)
        };

        let odr = regs::Odr::new_with_raw_value(0).with_odr(config.odr);
        spi_dev
            .start_transaction()
            .await
            .write_reg_u8(regs::Addr::Odr as u8, odr.raw_value());

        let osr = regs::Osr::new_with_raw_value(0)
            .with_osr_p(config.osr_p)
            .with_osr_t(config.osr_t);
        spi_dev
            .start_transaction()
            .await
            .write_reg_u8(regs::Addr::Osr as u8, osr.raw_value());

        let pwr_ctrl = regs::PwrCtrl::new_with_raw_value(0)
            .with_press_en(true)
            .with_temp_en(true)
            .with_mode(regs::PowerModeValue::Normal);

        spi_dev
            .start_transaction()
            .await
            .write_reg_u8(regs::Addr::PwrCtrl as u8, pwr_ctrl.raw_value());

        Ok(Bmp390 {
            spi_dev,
            compensation,
        })
    }

    fn check_odr(
        odr: regs::DataRateValue,
        osr_p: regs::OversamplingValue,
        osr_t: regs::OversamplingValue,
    ) -> bool {
        // See datasheet, section 3.9.2
        let tconv_us = 234 + (392 + osr_p as u32 * 2020) + (163 + osr_t as u32 * 2020);
        let fmax = 10000.0f32 / tconv_us as f32;

        let odr_sel: u8 = ceilf(log2f(200.0f32 / fmax)) as u8;

        (odr as u8) <= odr_sel
    }

    pub async fn sample(&mut self) -> Ts<Bmp390Sample> {
        let mut buf = [0 as u8; 6];

        self.spi_dev
            .start_transaction()
            .await
            .read_reg_raw(regs::Addr::PressData0 as u8, &mut buf);

        let ts = Instant::now().as_micros();
        let raw_temp = (buf[3] as u32) + ((buf[4] as u32) << 8) + ((buf[5] as u32) << 16);
        let raw_press = (buf[0] as u32) + ((buf[1] as u32) << 8) + ((buf[2] as u32) << 16);

        let temperature = self.compensation.temperature(raw_temp);
        let pressure = self.compensation.pressure(raw_press, temperature);

        let sample = Bmp390Sample {
            raw_press,
            raw_temp,
            value: PressureSensorSample {
                temperature: Some(temperature),
                pressure,
            },
        };

        Ts::from_microseconds(ts, sample)
    }
}
