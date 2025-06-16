use crate::{
    core::time::{Clock, Timestamp},
    crater::{
        channels,
        rocket::{
            mass::RocketMassProperties,
            rocket_data::{RocketAccelerations, RocketState},
        },
    },
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryReceiver, TelemetrySender, Timestamped},
    utils::capacity::Capacity::Unbounded,
};
use anyhow::Result;
use chrono::TimeDelta;
use crater_gnc::{DurationU64, datatypes::sensors::ImuSensorSample};
use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector4};
use rand_distr::{Distribution, Normal};
use rand_xoshiro::Xoshiro256StarStar;

#[derive(Debug)]
pub struct ImuParams {
    pos_r: Vector3<f64>,
    quat_imu_b: UnitQuaternion<f64>,
    imu_freq: f64,
    g_n: Vector3<f64>,
    std_acc_noise: f64,
    std_acc_bias: f64,
    acc_res_ms2: f64,
    acc_fs_range_g: f64,
    std_gyro_noise: f64,
    std_gyro_bias: f64,
    gyro_res_rads: f64,
    gyro_fs_range_dps: f64,
    rngs_acc: Vector3<Xoshiro256StarStar>,
    rngs_gyr: Vector3<Xoshiro256StarStar>,
}

/// Implementation of an Ideal IMU, without noise or errors
#[derive(Debug)]
pub struct IdealIMU {
    rx_state: TelemetryReceiver<RocketState>,
    rx_accels: TelemetryReceiver<RocketAccelerations>,
    rx_masses: TelemetryReceiver<RocketMassProperties>,
    params: ImuParams,
    step_count: usize,
    acc_bias: Vector3<f64>,
    gyro_bias: Vector3<f64>,
    tx_imu_translated: TelemetrySender<ImuSensorSample>,
    tx_imu_cg: TelemetrySender<ImuSensorSample>,
    tx_imu_real: TelemetrySender<ImuSensorSample>,
}

impl IdealIMU {
    pub fn new(ctx: NodeContext) -> Result<Self> {
        let rx_state = ctx
            .telemetry()
            .subscribe(channels::rocket::STATE, Unbounded)?;
        let rx_accels = ctx
            .telemetry()
            .subscribe(channels::rocket::ACCEL, Unbounded)?;
        let rx_masses = ctx
            .telemetry()
            .subscribe("/rocket/mass/rocket", Unbounded)?;

        let imu_params = ctx.parameters().get_map("sim.rocket.imu")?;

        let tx_imu_translated = ctx.telemetry().publish(channels::sensors::IDEAL_IMU)?;
        let tx_imu_cg = ctx.telemetry().publish(channels::sensors::IDEAL_IMU_CG)?;
        let tx_imu_real = ctx.telemetry().publish(channels::sensors::IMU_REAL)?;
        let pos_r = imu_params.get_param("pos_r")?.value_float_arr()?;
        let pos_r = Vector3::from_column_slice(&pos_r);

        let quat_imu_b = imu_params.get_param("quat_imu_b")?.value_float_arr()?;
        let quat_imu_b = UnitQuaternion::from_quaternion(Quaternion::from_vector(
            Vector4::from_column_slice(&quat_imu_b),
        ));

        let sim_dt = ctx.parameters().get_param("sim.dt")?.value_float()?;
        let imu_freq = imu_params.get_param("imu_freq")?.value_float()?;
        let imu_period = 1.0 / imu_freq;

        let ratio = imu_period / sim_dt;
        if (ratio - ratio.round()).abs() > 1e-8 {
            anyhow::bail!(
                "Simulation time step ({sim_dt}) is not a divisor of IMU period ({imu_period})"
            );
        }

        let g_n = ctx
            .parameters()
            .get_param("sim.rocket.g_n")?
            .value_float_arr()?;
        let g_n = Vector3::from_column_slice(&g_n);

        let mut std_acc_noise = imu_params.get_param("acc_noise")?.value_float()?;
        let mut std_acc_bias = imu_params.get_param("acc_bias")?.value_float()?;
        let acc_res_ms2 = imu_params.get_param("acc_res_ms2")?.value_float()?;
        let acc_fs_range_g = imu_params.get_param("acc_fs_range_g")?.value_float()?;

        let mut std_gyro_noise = imu_params.get_param("gyro_noise")?.value_float()?;
        let mut std_gyro_bias = imu_params.get_param("gyro_bias")?.value_float()?;
        let gyro_res_rads = imu_params.get_param("gyro_res_rads")?.value_float()?;
        let gyro_fs_range_dps = imu_params.get_param("gyro_fs_range_dps")?.value_float()?;

        std_acc_noise = std_acc_noise / imu_period.sqrt();
        std_acc_bias = std_acc_bias * imu_period.sqrt();
        std_gyro_noise = std_gyro_noise / imu_period.sqrt();
        std_gyro_bias = std_gyro_bias * imu_period.sqrt();

        let imu_parameters = ImuParams {
            pos_r,
            quat_imu_b,
            imu_freq,
            g_n,
            std_acc_noise,
            std_acc_bias,
            acc_res_ms2,
            acc_fs_range_g,
            std_gyro_noise,
            std_gyro_bias,
            gyro_res_rads,
            gyro_fs_range_dps,
            rngs_acc: Vector3::new(ctx.get_rng_256(), ctx.get_rng_256(), ctx.get_rng_256()),
            rngs_gyr: Vector3::new(ctx.get_rng_256(), ctx.get_rng_256(), ctx.get_rng_256()),
        };

        Ok(Self {
            rx_state,
            rx_accels,
            rx_masses,
            params: imu_parameters,
            step_count: 0,
            acc_bias: Vector3::zeros(),
            gyro_bias: Vector3::zeros(),
            tx_imu_translated,
            tx_imu_cg,
            tx_imu_real,
        })
    }
}

impl Node for IdealIMU {
    fn step(&mut self, _: usize, dt: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
        self.step_count += 1;

        let steps_per_imu = (1.0 / self.params.imu_freq / dt.as_seconds_f64()).round() as usize;

        let Timestamped(_, state) = self
            .rx_state
            .try_recv()
            .expect("IMU step executed, but no /rocket/state input available");
        let Timestamped(_, accel) = self
            .rx_accels
            .try_recv()
            .expect("IMU step executed, but no /rocket/actions input available");
        let Timestamped(_, masses) = self
            .rx_masses
            .try_recv()
            .expect("IMU step executed, but no /rocket/mass/rocket input available");

        if self.step_count < steps_per_imu {
            return Ok(StepResult::Continue);
        } else {
            self.step_count = 0;

            let imu_to_cg: Vector3<f64> = masses.xcg_total_m - self.params.pos_r;
            let angvel_b: Vector3<f64> = state.angvel_b_rad_s();

            // From: https://ocw.mit.edu/courses/16-07-dynamics-fall-2009/419be4d742e628d70acfbc5496eab967_MIT16_07F09_Lec25.pdf

            let meas_acc_cg_b: Vector3<f64> =
                accel.acc_b_m_s2 - state.quat_nb().inverse_transform_vector(&self.params.g_n);

            let meas_acc_b: Vector3<f64> = meas_acc_cg_b
                + accel.ang_acc_b_rad_s2.cross(&imu_to_cg)
                + angvel_b.cross(&angvel_b.cross(&imu_to_cg));

            let meas_acc_cg_imu: Vector3<f64> =
                self.params.quat_imu_b.transform_vector(&meas_acc_cg_b);
            let meas_acc_imu: Vector3<f64> = self.params.quat_imu_b.transform_vector(&meas_acc_b);

            let meas_angvel_imu: Vector3<f64> = self.params.quat_imu_b.transform_vector(&angvel_b);

            let accel_noise: Vector3<f64> = Vector3::from_fn(|_, _| {
                Normal::new(0.0, self.params.std_acc_noise)
                    .unwrap()
                    .sample(&mut self.params.rngs_acc.x)
            });

            let bias_acc: Vector3<f64> = Vector3::from_fn(|_, _| {
                Normal::new(0.0, self.params.std_acc_bias)
                    .unwrap()
                    .sample(&mut self.params.rngs_acc.y)
            });

            self.acc_bias += bias_acc;

            let meas_acc_real_imu: Vector3<f64> = meas_acc_imu + accel_noise + self.acc_bias;

            let gyro_noise: Vector3<f64> = Vector3::from_fn(|_, _| {
                Normal::new(0.0, self.params.std_gyro_noise)
                    .unwrap()
                    .sample(&mut self.params.rngs_gyr.x)
            });

            let bias_gyro: Vector3<f64> = Vector3::from_fn(|_, _| {
                Normal::new(0.0, self.params.std_gyro_bias)
                    .unwrap()
                    .sample(&mut self.params.rngs_gyr.y)
            });

            self.gyro_bias += bias_gyro;

            let meas_angvel_real_imu: Vector3<f64> = meas_angvel_imu + gyro_noise + self.gyro_bias;

            fn quantize_and_clip(v: f64, res: f64, max_val: f64) -> f32 {
                let clipped = v.clamp(-max_val, max_val);
                (res * (clipped / res).round()) as f32
            }

            let quantized_accel: Vector3<f32> = meas_acc_real_imu.map(|v| {
                quantize_and_clip(
                    v,
                    self.params.acc_res_ms2,
                    self.params.acc_fs_range_g * 9.8655,
                )
            });

            let quantized_gyro: Vector3<f32> = meas_angvel_real_imu.map(|v| {
                quantize_and_clip(
                    v,
                    self.params.gyro_res_rads,
                    self.params.gyro_fs_range_dps.to_radians(),
                )
            });

            self.tx_imu_cg.send(
                Timestamp::now(clock),
                ImuSensorSample {
                    accel_m_s2: meas_acc_cg_imu.map(|v| v as f32),
                    angvel_rad_s: meas_angvel_imu.map(|v| v as f32),
                    int_latency: DurationU64::micros(0).into(),
                    temperature_degc: None,
                    overrun_count: 0,
                },
            );

            self.tx_imu_translated.send(
                Timestamp::now(clock),
                ImuSensorSample {
                    accel_m_s2: meas_acc_imu.map(|v| v as f32),
                    angvel_rad_s: meas_angvel_imu.map(|v| v as f32),
                    int_latency: DurationU64::micros(0).into(),
                    temperature_degc: None,
                    overrun_count: 0,
                },
            );

            self.tx_imu_real.send(
                Timestamp::now(clock),
                ImuSensorSample {
                    accel_m_s2: quantized_accel.map(|v| v as f32),
                    angvel_rad_s: quantized_gyro.map(|v| v as f32),
                    int_latency: DurationU64::micros(0).into(),
                    temperature_degc: None,
                    overrun_count: 0,
                },
            );

            Ok(StepResult::Continue)
        }
    }
}
