use core::f32;

use crate::{
    Duration, DurationU64, Instant,
    mav_crater::{self, MavMessage, SensImuSample_DATA, SensPressureSample_DATA},
};
use nalgebra::Vector3;

#[derive(Debug, Clone)]
pub struct PressureSensorSample {
    pub pressure_pa: f32,
    pub temperature_degc: Option<f32>,
}

impl PressureSensorSample {
    pub fn to_mavlink(&self, id: mav_crater::PressureSensorId, ts: Instant) -> MavMessage {
        MavMessage::SensPressureSample(SensPressureSample_DATA {
            sensor_id: id,
            timestamp_us: ts.0.duration_since_epoch().to_micros() as i64,
            pressure_pa: self.pressure_pa,
            temperature_degc: self.temperature_degc.unwrap_or(f32::NAN),
        })
    }
}

impl From<SensPressureSample_DATA> for PressureSensorSample {
    fn from(data: SensPressureSample_DATA) -> Self {
        PressureSensorSample::from(&data)
    }
}

impl From<&SensPressureSample_DATA> for PressureSensorSample {
    fn from(data: &SensPressureSample_DATA) -> Self {
        Self {
            pressure_pa: data.pressure_pa,
            temperature_degc: if !data.temperature_degc.is_nan() {
                Some(data.temperature_degc)
            } else {
                None
            },
        }
    }
}

#[derive(Debug, Clone)]
pub struct ImuSensorSample {
    pub accel_m_s2: Vector3<f32>,
    pub angvel_rad_s: Vector3<f32>,
    pub temperature_degc: Option<f32>,
    pub int_latency: Duration,
    pub overrun_count: u8,
}

impl ImuSensorSample {
    pub fn to_mavlink(&self, id: mav_crater::ImuSensorId, ts: Instant) -> MavMessage {
        MavMessage::SensImuSample(SensImuSample_DATA {
            sensor_id: id,
            timestamp_us: ts.0.duration_since_epoch().to_micros() as i64,
            accel_m_s2: self.accel_m_s2.into(),
            ang_vel_deg_s: self.angvel_rad_s.into(),
            temperature_degc: self.temperature_degc.unwrap_or(f32::NAN),
            latency_us: self.int_latency.0.to_micros() as i64,
            overrun_count: self.overrun_count,
        })
    }
}

impl From<SensImuSample_DATA> for ImuSensorSample {
    fn from(data: SensImuSample_DATA) -> Self {
        ImuSensorSample::from(&data)
    }
}

impl From<&SensImuSample_DATA> for ImuSensorSample {
    fn from(data: &SensImuSample_DATA) -> Self {
        Self {
            accel_m_s2: data.accel_m_s2.into(),
            angvel_rad_s: data.ang_vel_deg_s.into(),
            temperature_degc: if !data.temperature_degc.is_nan() {
                Some(data.temperature_degc)
            } else {
                None
            },
            int_latency: DurationU64::micros(data.latency_us as u64).into(),
            overrun_count: data.overrun_count,
        }
    }
}

#[derive(Debug, Clone)]
pub struct GpsSensorSample {
    pub pos_n_m: Vector3<f32>,
    pub vel_n_m_s: Vector3<f32>,
}

#[derive(Debug, Clone)]
pub struct MagnetometerSensorSample {
    pub mag_field_b_gauss: Vector3<f32>,
}
