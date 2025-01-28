use crate::{
    core::time::{Clock, Timestamp},
    crater::sim::gnc::datatypes::ServoPosition,
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryDispatcher, TelemetrySender},
};
use anyhow::Result;
use chrono::TimeDelta;
use nalgebra::Vector4;

#[derive(Debug)]
pub struct OpenloopControl {
    tx_servo_cmd: TelemetrySender<ServoPosition>,
}

impl OpenloopControl {
    pub fn new(ctx: NodeContext) -> Result<Self> {
        let tx_servo_cmd = ctx.telemetry().publish("/gnc/control/servo_command")?;

        Ok(Self { tx_servo_cmd })
    }
}

impl Node for OpenloopControl {
    fn step(&mut self, _: usize, _: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
        let cmd = ServoPosition {
            servo_positions: Vector4::<f64>::zeros(),
        };

        self.tx_servo_cmd.send(Timestamp::now(clock), cmd);

        Ok(StepResult::Continue)
    }
}
