use anyhow::Result;
use chrono::TimeDelta;
use crate::{
    core::time::{Clock, Timestamp},
    crater::sim::{
        rocket_data::{RocketActions, RocketState},
        sensors::datatypes::IMUSample,
    },
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryDispatcher, TelemetryReceiver, TelemetrySender, Timestamped},
    utils::capacity::Capacity::Unbounded,
};

/// Implementation of an Ideal IMU, without noise or errors
/// TODO: Implement measurements offset from the CG
#[derive(Debug)]
pub struct IdealIMU {
    rx_state: TelemetryReceiver<RocketState>,
    rx_actions: TelemetryReceiver<RocketActions>,

    tx_imu: TelemetrySender<IMUSample>,
}

impl IdealIMU {
    pub fn new(ctx: NodeContext) -> Result<Self> {
        let rx_state = ctx.telemetry().subscribe("/rocket/state", Unbounded)?;
        let rx_actions = ctx.telemetry().subscribe("/rocket/actions", Unbounded)?;

        let tx_imu = ctx.telemetry().publish("/sensors/imu")?;

        Ok(Self {
            rx_state,
            rx_actions,
            tx_imu,
        })
    }
}

impl Node for IdealIMU {
    fn step(&mut self, _: usize, _: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
        let Timestamped(_, state) = self.rx_state.try_recv().expect("IMU step executed, but no /rocket/state input available");
        let Timestamped(_, actions) = self.rx_actions.try_recv().expect("IMU step executed, but no /rocket/actions input available");

        let sample = IMUSample {
            acc: actions.acc_b,
            gyro: state.angvel_b()
        };

        self.tx_imu.send(Timestamp::now(clock), sample);

        Ok(StepResult::Continue)
    }
}
