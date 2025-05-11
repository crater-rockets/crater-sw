use crate::{
    core::time::{Clock, Timestamp},
    crater::sim::{
        rocket_data::{RocketAccelerations, RocketActions, RocketMassProperties, RocketState},
        sensors::datatypes::IMUSample,
    },
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryReceiver, TelemetrySender, Timestamped},
    utils::capacity::Capacity::Unbounded,
};
use anyhow::Result;
use chrono::TimeDelta;
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
    tx_imu_translated: TelemetrySender<IMUSample>,
    tx_imu_cg: TelemetrySender<IMUSample>,
}

impl IdealIMU {
    pub fn new(ctx: NodeContext) -> Result<Self> {
        let rx_state = ctx.telemetry().subscribe("/rocket/state", Unbounded)?;
        let rx_accels = ctx.telemetry().subscribe("/rocket/accel", Unbounded)?;
        let rx_masses = ctx.telemetry().subscribe("/rocket/masses", Unbounded)?;

        let imu_params = ctx.parameters().get_map("sim.rocket.imu")?;

        let tx_imu_translated = ctx.telemetry().publish("/sensors/ideal_imu/translated")?;
        let tx_imu_cg = ctx.telemetry().publish("/sensors/ideal_imu/cg")?;

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
            .expect("IMU step executed, but no /rocket/masses input available");

        let imu_to_cg = masses.xcg_total - self.params.pos_r;
        let angvel_b = state.angvel_b();

        // From: https://ocw.mit.edu/courses/16-07-dynamics-fall-2009/419be4d742e628d70acfbc5496eab967_MIT16_07F09_Lec25.pdf

        let meas_acc_cg_b =
            accel.acc_b - state.quat_nb().inverse_transform_vector(&self.params.g_n);

        let meas_acc_b: Vector3<f64> = meas_acc_cg_b
            + accel.ang_acc_b.cross(&imu_to_cg)
            + angvel_b.cross(&angvel_b.cross(&imu_to_cg));

        let meas_acc_cg_imu = self.params.quat_imu_b.transform_vector(&meas_acc_cg_b);
        let meas_acc_imu = self.params.quat_imu_b.transform_vector(&meas_acc_b);

        let meas_angvel_imu: Vector3<f64> = self.params.quat_imu_b.transform_vector(&angvel_b);

        self.tx_imu_cg.send(
            Timestamp::now(clock),
            IMUSample {
                acc: meas_acc_cg_imu,
                gyro: meas_angvel_imu,
            },
        );

        self.tx_imu_translated.send(
            Timestamp::now(clock),
            IMUSample {
                acc: meas_acc_imu,
                gyro: meas_angvel_imu,
            },
        );

        Ok(StepResult::Continue)
    }
}
