use core::{array, f32};

use nalgebra::Vector3;
use uom::si::{
    acceleration::meter_per_second_squared,
    angular_velocity::{degree_per_second, radian_per_second},
    f32::{Acceleration, AngularVelocity, Pressure, ThermodynamicTemperature},
    pressure::pascal,
    thermodynamic_temperature::degree_celsius,
};

use crate::{
    Instant,
    mav_crater::{self, MavMessage, SensImuSample_DATA, SensPressureSample_DATA},
};

#[derive(Debug, Clone)]
pub struct PressureSensorSample {
    pub pressure: uom::si::f32::Pressure,
    pub temperature: Option<uom::si::f32::ThermodynamicTemperature>,
}

impl PressureSensorSample {
    pub fn to_mavlink(&self, id: mav_crater::PressureSensorId, ts: Instant) -> MavMessage {
        MavMessage::SensPressureSample(SensPressureSample_DATA {
            sensor_id: id,
            timestamp_us: ts.0.duration_since_epoch().to_micros() as i64,
            pressure_pa: self.pressure.get::<pascal>(),
            temperature_degc: self
                .temperature
                .map(|v| v.get::<degree_celsius>())
                .unwrap_or(f32::NAN),
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
            pressure: Pressure::new::<pascal>(data.pressure_pa),
            temperature: if !data.temperature_degc.is_nan() {
                Some(ThermodynamicTemperature::new::<degree_celsius>(
                    data.temperature_degc,
                ))
            } else {
                None
            },
        }
    }
}

#[derive(Debug, Clone)]
pub struct ImuSensorSample {
    pub accel: [uom::si::f32::Acceleration; 3],
    pub ang_vel: [uom::si::f32::AngularVelocity; 3],
    pub temperature: Option<uom::si::f32::ThermodynamicTemperature>,
}

impl ImuSensorSample {
    pub fn accel_m_s2_array(&self) -> [f32; 3] {
        array::from_fn(|i| self.accel[i].get::<meter_per_second_squared>())
    }

    pub fn ang_vel_rad_s_array(&self) -> [f32; 3] {
        array::from_fn(|i| self.ang_vel[i].get::<radian_per_second>())
    }

    pub fn ang_vel_deg_s_array(&self) -> [f32; 3] {
        array::from_fn(|i| self.ang_vel[i].get::<degree_per_second>())
    }

    pub fn accel_m_s2_vec(&self) -> Vector3<f32> {
        Vector3::from_fn(|i, _| self.accel[i].get::<meter_per_second_squared>())
    }

    pub fn ang_vel_rad_s_vec(&self) -> Vector3<f32> {
        Vector3::from_fn(|i, _| self.ang_vel[i].get::<radian_per_second>())
    }

    pub fn ang_vel_deg_s_vec(&self) -> Vector3<f32> {
        Vector3::from_fn(|i, _| self.ang_vel[i].get::<degree_per_second>())
    }

    pub fn to_mavlink(&self, id: mav_crater::ImuSensorId, ts: Instant) -> MavMessage {
        MavMessage::SensImuSample(SensImuSample_DATA {
            sensor_id: id,
            timestamp_us: ts.0.duration_since_epoch().to_micros() as i64,
            accel_m_s2: self.accel_m_s2_array(),
            ang_vel_rad_s: self.ang_vel_rad_s_array(),
            temperature_degc: self
                .temperature
                .map(|v| v.get::<degree_celsius>())
                .unwrap_or(f32::NAN),
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
            accel: array::from_fn(|i| {
                Acceleration::new::<meter_per_second_squared>(data.accel_m_s2[i])
            }),
            ang_vel: array::from_fn(|i| {
                AngularVelocity::new::<radian_per_second>(data.ang_vel_rad_s[i])
            }),
            temperature: if !data.temperature_degc.is_nan() {
                Some(ThermodynamicTemperature::new::<degree_celsius>(
                    data.temperature_degc,
                ))
            } else {
                None
            },
        }
    }
}
