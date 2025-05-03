use crate::{
    core::time::{Clock, Timestamp},
    crater::sim::{
        rocket_data::{RocketActions, RocketMassProperties, RocketState},
        sensors::datatypes::IMUSample,
    },
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryDispatcher, TelemetryReceiver, TelemetrySender, Timestamped},
    utils::capacity::Capacity::Unbounded,
};
use anyhow::Result;
use chrono::TimeDelta;
use nalgebra::{Matrix3, Vector3};

#[derive(Debug)]
pub struct ImuParams {
    imu_pos: Vector3<f64>,
    imu_rot: Matrix3<f64>,
    imu_mis: Matrix3<f64>,
    g_n: Vector3<f64>,
}

/// Implementation of an Ideal IMU, without noise or errors
/// TODO: Implement measurements offset from the CG
#[derive(Debug)]
pub struct IdealIMU {
    rx_state: TelemetryReceiver<RocketState>,
    rx_actions: TelemetryReceiver<RocketActions>,
    rx_masses: TelemetryReceiver<RocketMassProperties>,
    imu_parameters: ImuParams,
    tx_imu: TelemetrySender<IMUSample>,
}

impl IdealIMU {
    pub fn new(ctx: NodeContext) -> Result<Self> {
        let rx_state = ctx.telemetry().subscribe("/rocket/state", Unbounded)?;
        let rx_actions = ctx.telemetry().subscribe("/rocket/actions", Unbounded)?;
        let rx_masses = ctx.telemetry().subscribe("/rocket/masses", Unbounded)?;

        let imu_params = ctx.parameters().get_map("sim.rocket.imu")?;

        let tx_imu = ctx.telemetry().publish("/sensors/ideal_imu")?;

        let imu_pos = imu_params.get_param("imu_position")?.value_float_arr()?;
        let imu_pos = Vector3::from_column_slice(&imu_pos);

        let imu_rot = imu_params.get_param("imu_rotation")?.value_float_arr()?;
        let imu_rot = Matrix3::from_column_slice(&imu_rot);

        let imu_mis = imu_params.get_param("imu_misalign")?.value_float_arr()?;
        let imu_mis = Matrix3::from_column_slice(&imu_mis);

        let g_n = ctx
            .parameters()
            .get_param("sim.rocket.g_n")?
            .value_float_arr()?;
        let g_n = Vector3::from_column_slice(&g_n);

        let imu_parameters = ImuParams {
            imu_pos,
            imu_rot,
            imu_mis,
            g_n,
        };

        Ok(Self {
            rx_state,
            rx_actions,
            rx_masses,
            imu_parameters,
            tx_imu,
        })
    }
}

impl Node for IdealIMU {
    fn step(&mut self, _: usize, _: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
        let Timestamped(_, state) = self
            .rx_state
            .try_recv()
            .expect("IMU step executed, but no /rocket/state input available");
        let Timestamped(_, actions) = self
            .rx_actions
            .try_recv()
            .expect("IMU step executed, but no /rocket/actions input available");
        let Timestamped(_, masses) = self
            .rx_masses
            .try_recv()
            .expect("IMU step executed, but no /rocket/masses input available");

        let cg_to_imu = self.imu_parameters.imu_pos - masses.xcg_total;
        let angvel_b = state.angvel_b();

        let acc_imu_pos: Vector3<f64> = actions.acc_b
            + actions.ang_acc_b.cross(&cg_to_imu)
            + angvel_b.cross(&angvel_b.cross(&cg_to_imu))
            - state
                .quat_nb()
                .inverse_transform_vector(&self.imu_parameters.g_n);

        let acc_rotated = self.imu_parameters.imu_mis * self.imu_parameters.imu_rot * acc_imu_pos;

        let w_imu: Vector3<f64> =
            self.imu_parameters.imu_mis * self.imu_parameters.imu_rot * state.angvel_b();

        let sample = IMUSample {
            acc: acc_rotated,
            gyro: w_imu,
        };

        self.tx_imu.send(Timestamp::now(clock), sample);

        Ok(StepResult::Continue)
    }
}
