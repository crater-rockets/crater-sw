use crate::{
    core::time::{Clock, Timestamp},
    crater::{
        channels,
        rocket::rocket_data::RocketState,
    },
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryReceiver, TelemetrySender, Timestamped},
    utils::capacity::Capacity::Unbounded,
};

use anyhow::Result;
use chrono::TimeDelta;
use nalgebra::Vector3;
use rand::rngs::StdRng;
use rand_xoshiro::{Xoroshiro64StarStar, Xoshiro256StarStar};

#[derive(Debug, Clone)]
pub struct WindSample {
    pub wind_vel_ned: Vector3<f64>,
    pub wind_ang_vel: Vector3<f64>,
    pub wind_gust: Vector3<f64>,
    pub wind_clean: Vector3<f64>,
}

#[derive(Debug, Clone)]
pub enum Turbulence {
    CONST = 0,
    MIL1797 = 1,
    MIL8785 = 2,
}
#[derive(Debug)]
pub struct WindParams {
    pub wind_20_feet: f64,
    pub wind_vel_0: Vector3<f64>,
    pub turb_intensity: f64,
    pub turbulence: Turbulence,
}

#[derive(Debug)]
pub struct WindData {
    wind_old: Vector3<f64>,
    gust_old: Vector3<f64>,
    scale_length: Vector3<f64>,
    sigmas: Vector3<f64>,
    rngs: Vector3<Xoshiro256StarStar>,
}

#[derive(Debug)]
pub struct WindModel {
    rx_state: TelemetryReceiver<RocketState>,
    tx_wind: TelemetrySender<WindSample>,
    params: WindParams,
    data: WindData,
}


impl WindModel{
    pub fn new(ctx: NodeContext) -> Result<Self> {

        let rx_state = ctx.telemetry().subscribe(channels::rocket::STATE, Unbounded)?;

        let wind_params = ctx.parameters().get_map("sim.rocket.wind")?;

        let tx_wind = ctx.telemetry().publish(channels::sim::WIND)?;

        let wind_vel = wind_params.get_param("wind_vel")?.value_float()?;
        let azimuth = wind_params.get_param("azimuth")?.value_float()?;
        let wind_20_feet = wind_params.get_param("wind_20_feet")?.value_float()?;
        let turb_intensity = wind_params.get_param("turb_intensity")?.value_float()?;
        let turb_type = wind_params.get_param("turb_type")?.value_int()?;

        let rng_u: Xoshiro256StarStar = ctx.get_rng_256();
        let rng_v: Xoshiro256StarStar = ctx.get_rng_256();
        let rng_w: Xoshiro256StarStar = ctx.get_rng_256();

        let turbulence = match turb_type {
            0 => Turbulence::CONST,
            1 => Turbulence::MIL8785,
            _ => Turbulence::CONST,
        };

        let wind_vel_0 = Vector3::new(wind_vel*azimuth.cos(), wind_vel*azimuth.sin(), 0.0);

        let params = WindParams{
            wind_20_feet,
            wind_vel_0,
            turb_intensity,
            turbulence,
        };

        let data = WindData{
            wind_old: wind_vel_0,
            gust_old: Vector3::zeros(),
            scale_length: Vector3::zeros(),
            sigmas: Vector3::zeros(),
            rngs: Vector3::new(rng_u,rng_v,rng_w),
        };
        
        Ok(Self{
            rx_state,
            tx_wind,
            params,
            data
        })
    }
}

impl Node for WindModel{
    fn step(&mut self, i: usize, dt: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
        Ok(StepResult::Continue)
    }
}