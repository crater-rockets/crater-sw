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

#[derive(Debug)]
pub struct ImuParams {
    pos_r: Vector3<f64>,
    quat_imu_b: UnitQuaternion<f64>,
    g_n: Vector3<f64>,
}

/// Implementation of an Ideal IMU, without noise or errors
#[derive(Debug)]
pub struct IdealIMU {
    rx_state: TelemetryReceiver<RocketState>,
    rx_accels: TelemetryReceiver<RocketAccelerations>,
    rx_masses: TelemetryReceiver<RocketMassProperties>,
    params: ImuParams,
    tx_imu_translated: TelemetrySender<ImuSensorSample>,
    tx_imu_cg: TelemetrySender<ImuSensorSample>,
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

        let pos_r = imu_params.get_param("pos_r")?.value_float_arr()?;
        let pos_r = Vector3::from_column_slice(&pos_r);

        let quat_imu_b = imu_params.get_param("quat_imu_b")?.value_float_arr()?;
        let quat_imu_b = UnitQuaternion::from_quaternion(Quaternion::from_vector(
            Vector4::from_column_slice(&quat_imu_b),
        ));

        let g_n = ctx
            .parameters()
            .get_param("sim.rocket.g_n")?
            .value_float_arr()?;
        let g_n = Vector3::from_column_slice(&g_n);

        let imu_parameters = ImuParams {
            pos_r,
            quat_imu_b,
            g_n,
        };

        Ok(Self {
            rx_state,
            rx_accels,
            rx_masses,
            params: imu_parameters,
            tx_imu_translated,
            tx_imu_cg,
        })
    }
}

impl Node for IdealIMU {
    fn step(&mut self, _: usize, _: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
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

        let imu_to_cg = masses.xcg_total_m - self.params.pos_r;
        let angvel_b = state.angvel_b_rad_s();

        // From: https://ocw.mit.edu/courses/16-07-dynamics-fall-2009/419be4d742e628d70acfbc5496eab967_MIT16_07F09_Lec25.pdf

        let meas_acc_cg_b =
            accel.acc_b_m_s2 - state.quat_nb().inverse_transform_vector(&self.params.g_n);

        let meas_acc_b: Vector3<f64> = meas_acc_cg_b
            + accel.ang_acc_b_rad_s2.cross(&imu_to_cg)
            + angvel_b.cross(&angvel_b.cross(&imu_to_cg));

        let meas_acc_cg_imu = self.params.quat_imu_b.transform_vector(&meas_acc_cg_b);
        let meas_acc_imu = self.params.quat_imu_b.transform_vector(&meas_acc_b);

        let meas_angvel_imu: Vector3<f64> = self.params.quat_imu_b.transform_vector(&angvel_b);

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

        Ok(StepResult::Continue)
    }
}
