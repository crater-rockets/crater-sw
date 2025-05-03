use crate::{
    core::time::{Clock, Timestamp},
    crater::sim::{
        rocket_data::{RocketParams, RocketState},
        sensors::datatypes::MagnetometerSample,
    },
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryDispatcher, TelemetryReceiver, TelemetrySender, Timestamped},
    utils::capacity::Capacity::Unbounded,
};
use anyhow::Result;
use chrono::TimeDelta;
use map_3d::ned2geodetic;
use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3, Vector4};
use num_traits::ToPrimitive;
use world_magnetic_model::{
    time::OffsetDateTime,
    uom::si::{
        f32::{Angle, Length},
        length::meter,
        magnetic_flux_density::nanotesla,
    },
};
use world_magnetic_model::{uom::si::angle::radian, GeomagneticField};

#[derive(Debug)]
pub struct MagParams {
    quat_mag_b: UnitQuaternion<f64>,
}

#[derive(Debug)]
pub struct IdealMagnetometer {
    rx_state: TelemetryReceiver<RocketState>,
    rx_params: TelemetryReceiver<RocketParams>,
    tx_magn: TelemetrySender<MagnetometerSample>,
    mag_par: MagParams,
}

impl IdealMagnetometer {
    pub fn new(ctx: NodeContext) -> Result<Self> {
        let rx_state = ctx.telemetry().subscribe("/rocket/state", Unbounded)?;
        let rx_params = ctx.telemetry().subscribe("/rocket/params", Unbounded)?;
        let tx_magn = ctx.telemetry().publish("/sensors/ideal_mag")?;

        let mag_params = ctx.parameters().get_map("sim.rocket.magnetomer")?;

        let quat_mag_b = mag_params.get_param("quat_mag_b")?.value_float_arr()?;
        let quat_mag_b = UnitQuaternion::from_quaternion(Quaternion::from_vector(
            Vector4::from_column_slice(&quat_mag_b),
        ));

        let mag_par: MagParams = MagParams { quat_mag_b };

        Ok(Self {
            rx_state,
            rx_params,
            tx_magn,
            mag_par,
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

        let Timestamped(_, params) = self
            .rx_params
            .try_recv()
            .expect("Magnetometer step executed, but no /rocket/params input available");

        let pos = state.pos_n();

        let (lat, lon, _) = ned2geodetic(
            pos[0],
            pos[1],
            pos[2],
            params.origin_geo[0],
            params.origin_geo[1],
            params.origin_geo[2],
            map_3d::Ellipsoid::WGS84,
        );

        let alt = (-state.pos_n().z + params.origin_geo.z).to_f32().unwrap();
        let today = OffsetDateTime::now_utc().date();

        let mag_field = GeomagneticField::new(
            Length::new::<meter>(alt),
            Angle::new::<radian>(lat.to_f32().unwrap()),
            Angle::new::<radian>(lon.to_f32().unwrap()),
            today,
        )
        .unwrap();

        let mag_n = mag_field.x().get::<nanotesla>().to_f64().unwrap();
        let mag_e = mag_field.y().get::<nanotesla>().to_f64().unwrap();
        let mag_d = mag_field.z().get::<nanotesla>().to_f64().unwrap();

        let mag_ned: Vector3<f64> = Vector3::new(mag_n, mag_e, mag_n);

        let sample = MagnetometerSample {
            magfield_b: self
                .mag_par
                .quat_mag_b
                .transform_vector(&state.quat_nb().inverse_transform_vector(&mag_ned)),
        };

        self.tx_magn.send(Timestamp::now(clock), sample);

        Ok(StepResult::Continue)
    }
}
