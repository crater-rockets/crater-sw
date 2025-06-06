use crate::{
    core::time::{Clock, Timestamp},
    crater::{channels, rocket::rocket_data::RocketState},
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryReceiver, TelemetrySender, Timestamped},
    utils::capacity::Capacity::Unbounded,
};
use anyhow::Result;
use chrono::TimeDelta;
use crater_gnc::datatypes::sensors::MagnetometerSensorSample;
use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector4};
use num_traits::ToPrimitive;
use world_magnetic_model::{GeomagneticField, uom::si::angle::radian};
use world_magnetic_model::{
    time::{Date, macros::format_description},
    uom::si::{
        f32::{Angle, Length},
        length::meter,
        magnetic_flux_density::nanotesla,
    },
};

#[derive(Debug)]
pub struct MagParams {
    quat_mag_b: UnitQuaternion<f64>,
}

#[derive(Debug)]
pub struct IdealMagnetometer {
    rx_state: TelemetryReceiver<RocketState>,
    tx_magn: TelemetrySender<MagnetometerSensorSample>,
    mag_par: MagParams,
    mag_ned: Vector3<f64>,
}

impl IdealMagnetometer {
    pub fn new(ctx: NodeContext) -> Result<Self> {
        let rx_state = ctx
            .telemetry()
            .subscribe(channels::rocket::STATE, Unbounded)?;
        let tx_magn = ctx
            .telemetry()
            .publish(channels::sensors::IDEAL_MAGNETOMETER)?;

        let mag_params = ctx.parameters().get_map("sim.rocket.magnetomer")?;

        let quat_mag_b = mag_params.get_param("quat_mag_b")?.value_float_arr()?;
        let quat_mag_b = UnitQuaternion::from_quaternion(Quaternion::from_vector(
            Vector4::from_column_slice(&quat_mag_b),
        ));

        let mag_par: MagParams = MagParams { quat_mag_b };

        let latitude_rad = ctx
            .parameters()
            .get_param("sim.rocket.init.latitude")?
            .value_float()?
            .to_radians();
        let longitude_rad = ctx
            .parameters()
            .get_param("sim.rocket.init.longitude")?
            .value_float()?
            .to_radians();
        let altitude_m = ctx
            .parameters()
            .get_param("sim.rocket.init.altitude")?
            .value_float()?;
        let date_str = ctx
            .parameters()
            .get_param("sim.rocket.date")?
            .value_string()?;

        let format = format_description!("[year]-[month]-[day]");
        let date = Date::parse(&date_str, format)?;

        let mag_field = GeomagneticField::new(
            Length::new::<meter>(altitude_m as f32),
            Angle::new::<radian>(latitude_rad as f32),
            Angle::new::<radian>(longitude_rad as f32),
            date,
        )
        .unwrap();

        let mag_n = mag_field.x().get::<nanotesla>().to_f64().unwrap();
        let mag_e = mag_field.y().get::<nanotesla>().to_f64().unwrap();
        let mag_d = mag_field.z().get::<nanotesla>().to_f64().unwrap();

        let mag_ned: Vector3<f64> = Vector3::new(mag_n, mag_e, mag_d);

        Ok(Self {
            rx_state,
            tx_magn,
            mag_par,
            mag_ned,
        })
    }
}

impl Node for IdealMagnetometer {
    #[allow(unused)]
    fn step(&mut self, _: usize, _: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
        let Timestamped(_, state) = self
            .rx_state
            .try_recv()
            .expect("Magnetometer step executed, but no /rocket/state input available");

        let sample = MagnetometerSensorSample {
            mag_field_b_gauss: self
                .mag_par
                .quat_mag_b
                .transform_vector(&state.quat_nb().inverse_transform_vector(&self.mag_ned))
                .map(|v| v as f32),
        };

        self.tx_magn.send(Timestamp::now(clock), sample);

        Ok(StepResult::Continue)
    }
}
