use crate::math::interp::Interpolator;
use crate::{
    core::time::{Clock, Timestamp},
    crater::{channels, rocket::rocket_data::RocketState},
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryReceiver, TelemetrySender, Timestamped},
    utils::capacity::Capacity::Unbounded,
};
use anyhow::Result;
use chrono::TimeDelta;
use nalgebra::Vector3;
use nalgebra::coordinates::M2x2;
use rand_distr::{Distribution, Normal};
use rand_xoshiro::Xoshiro256StarStar;
#[derive(Debug, Clone, Default)]
pub struct WindSample {
    pub wind_vel_ned: Vector3<f64>,
    pub wind_ang_vel: Vector3<f64>,
    pub wind_gust: Vector3<f64>,
    pub wind_clean: Vector3<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Turbulence {
    CONST = 0,
    MIL8785 = 1,
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

pub struct WindModel {
    rx_state: TelemetryReceiver<RocketState>,
    tx_wind: TelemetrySender<WindSample>,
    params: WindParams,
    data: WindData,
    interpolator: Interpolator<f64, 2>,
    table_data: Vec<f64>,
}

impl WindModel {
    pub fn new(ctx: NodeContext) -> Result<Self> {
        let rx_state = ctx
            .telemetry()
            .subscribe(channels::rocket::STATE, Unbounded)?;
        let wind_params = ctx.parameters().get_map("sim.rocket.wind")?;
        let tx_wind = ctx.telemetry().publish(channels::sim::WIND)?;

        let wind_vel = wind_params.get_param("wind_vel")?.value_float()?;
        let azimuth = wind_params.get_param("azimuth")?.value_float()?;
        let wind_20_feet = wind_params.get_param("wind_20_feet")?.value_float()?;
        let turb_intensity = wind_params.get_param("turb_intensity")?.value_float()?;
        let turb_type = wind_params.get_param("turb_type")?.value_int()?;

        let rng_u = ctx.get_rng_256();
        let rng_v = ctx.get_rng_256();
        let rng_w = ctx.get_rng_256();

        let turbulence = match turb_type {
            1 => Turbulence::MIL8785,
            _ => Turbulence::CONST,
        };

        let wind_vel_0 = Vector3::new(wind_vel * azimuth.cos(), wind_vel * azimuth.sin(), 0.0);
        let params = WindParams {
            wind_20_feet,
            wind_vel_0,
            turb_intensity,
            turbulence,
        };

        let data = WindData {
            wind_old: wind_vel_0,
            gust_old: Vector3::zeros(),
            scale_length: Vector3::zeros(),
            sigmas: Vector3::zeros(),
            rngs: Vector3::new(rng_u, rng_v, rng_w),
        };

        let altitudes = [
            2000.0, 3805.402, 7573.307, 25070.916, 35135.591, 44876.188, 54937.621, 65107.441,
            74221.927, 80000.0,
        ];
        let turbulence_levels = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0];

        let table_data = vec![
            2.077, 3.523, 7.018, 9.662, 13.140, 17.663, 21.899, 1.421, 3.340, 7.340, 10.514,
            15.810, 23.044, 28.434, 0.000, 1.551, 6.710, 10.119, 15.026, 23.622, 30.240, 0.000,
            0.000, 2.619, 6.521, 9.812, 20.046, 31.007, 0.000, 0.000, 0.372, 5.026, 8.129, 15.929,
            25.182, 0.000, 0.000, 0.007, 4.254, 8.167, 15.134, 23.211, 0.000, 0.000, 0.000, 2.692,
            7.910, 12.100, 17.538, 0.000, 0.000, 0.000, 0.002, 4.932, 7.838, 10.701, 0.000, 0.000,
            0.000, 0.000, 3.412, 6.321, 8.570, 0.000, 0.000, 0.000, 0.000, 2.130, 5.104, 7.219,
        ];

        let interpolator = Interpolator::<f64, 2>::new([&altitudes, &turbulence_levels]).unwrap();

        Ok(Self {
            rx_state,
            tx_wind,
            params,
            data,
            interpolator,
            table_data,
        })
    }

    fn scale_lengths(&mut self, altitude: f64) {
        let mut alt_ft = altitude * 3.28084;
        if altitude <= 1.0 {
            alt_ft = 1.0;
        }

        let (length_u, length_v, length_w) = if self.params.turbulence == Turbulence::MIL8785 {
            if alt_ft <= 1000.0 {
                let l = alt_ft / (0.177 + 0.000823 * alt_ft).powf(1.2);
                (l, l, alt_ft)
            } else if alt_ft <= 2000.0 {
                let base = 1000.0_f64 / (0.177_f64 + 0.000823_f64 * 1000.0_f64).powf(1.2);
                let delta = (1750.0 - base) / 1000.0 * (alt_ft - 1000.0);
                (base + delta, base + delta, base + delta)
            } else {
                (1750.0, 1750.0, 1750.0)
            }
        } else {
            (0.0, 0.0, 0.0)
        };

        self.data.scale_length = Vector3::new(length_u, length_v, length_w);
    }

    fn turb_intensity(&mut self, altitude: f64) {
        let alt_ft = altitude * 3.28084;

        let sigma_z_standard = 0.1 * self.params.wind_20_feet;
        let sigma_u_standard = 1.0 / (0.177 + 0.000823 * alt_ft).powf(0.4) * sigma_z_standard;

        if alt_ft <= 1000.0 {
            self.data.sigmas.z = sigma_z_standard;
            self.data.sigmas.x = sigma_u_standard;
            self.data.sigmas.y = sigma_u_standard;
        } else if alt_ft < 2000.0 {
            let t = (alt_ft - 1000.0) / 1000.0;
            let mut output: [f64; 1] = [0.0];
            let state: [f64; 2] = [2000.0, self.params.turb_intensity];
            self.interpolator
                .interpn(&state, &[&self.table_data], &mut output);
            self.data.sigmas.z = (1.0 - t) * sigma_z_standard + t * output[0];
            self.data.sigmas.x = (1.0 - t) * sigma_u_standard + t * output[0];
            self.data.sigmas.y = self.data.sigmas.x;
        } else {
            let state = [altitude, self.params.turb_intensity];
            let mut output = [0.0];
            self.interpolator
                .interpn(&state, &[&self.table_data], &mut output);
            self.data.sigmas.z = output[0];
            self.data.sigmas.x = output[0];
            self.data.sigmas.y = output[0];
        }
    }

    fn calc_wind(&mut self, vel_body: f64, dt: f64) {
        let mut vel = vel_body * 3.28084;
        if self.params.turbulence == Turbulence::CONST {
            self.data.wind_old = self.params.wind_vel_0;
        } else {
            if vel <= 0.0 {
                vel = 0.0;
            }

            let a1_u = -vel / self.data.scale_length.x;
            let b1_u = self.data.sigmas.x * (2.0 * vel / self.data.scale_length.x).sqrt();

            let a1_v = -vel / self.data.scale_length.y;
            let b1_v = self.data.sigmas.y * (2.0 * vel / self.data.scale_length.y).sqrt();

            let a1_w = -vel / self.data.scale_length.z;
            let b1_w = self.data.sigmas.z * (2.0 * vel / self.data.scale_length.z).sqrt();

            let eta_u = Normal::new(0.0, 1.0).unwrap().sample(&mut self.data.rngs.x);
            let eta_v = Normal::new(0.0, 1.0).unwrap().sample(&mut self.data.rngs.y);
            let eta_w = Normal::new(0.0, 1.0).unwrap().sample(&mut self.data.rngs.z);

            let rad_dt = dt.sqrt();

            self.data.gust_old.x =
                self.data.gust_old.x + a1_u * self.data.gust_old.x * dt + b1_u * rad_dt * eta_u;
            self.data.gust_old.y =
                self.data.gust_old.y + a1_v * self.data.gust_old.y * dt + b1_v * rad_dt * eta_v;
            self.data.gust_old.z =
                self.data.gust_old.z + a1_w * self.data.gust_old.z * dt + b1_w * rad_dt * eta_w;

            self.data.wind_old = self.params.wind_vel_0 + self.data.gust_old;
        }
    }
}

impl Node for WindModel {
    fn step(&mut self, _i: usize, dt: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
        let Timestamped(_, state) = self
            .rx_state
            .try_recv()
            .expect("Wind step executed, but no /rocket/state input available");

        self.scale_lengths(-state.pos_n_m().z);

        self.turb_intensity(-state.pos_n_m().z);

        self.calc_wind(state.vel_n_m_s().norm(), dt.as_seconds_f64());

        self.tx_wind.send(
            Timestamp::now(clock),
            WindSample {
                wind_vel_ned: self.data.wind_old,
                wind_ang_vel: self.data.gust_old,
                wind_gust: self.data.gust_old,
                wind_clean: self.params.wind_vel_0,
            },
        );

        Ok(StepResult::Continue)
    }
}
