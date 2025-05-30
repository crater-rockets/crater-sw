use crate::{
    core::time::{Clock, Timestamp},
    crater::{channels, gnc::ServoPosition},
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryReceiver, TelemetrySender, Timestamped},
    utils::capacity::Capacity::Unbounded,
};
use anyhow::Result;
use chrono::TimeDelta;

#[derive(Debug)]
pub struct IdealServo {
    rx_control: TelemetryReceiver<ServoPosition>,
    tx_servo_pos: TelemetrySender<ServoPosition>,
}

impl IdealServo {
    pub fn new(ctx: NodeContext) -> Result<Self> {
        let rx_control = ctx
            .telemetry()
            .subscribe(channels::gnc::SERVO_COMMAND, Unbounded)?;

        let tx_servo_pos = ctx
            .telemetry()
            .publish(channels::actuators::IDEAL_SERVO_POSITION)?;

        Ok(Self {
            rx_control,
            tx_servo_pos,
        })
    }
}

impl Node for IdealServo {
    fn step(&mut self, _: usize, _: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
        let Timestamped(_, cmd) = self
            .rx_control
            .try_recv()
            .expect("IdealServo step executed, but no /gnc/control/servo_command input available");

        // Just repeat the command
        self.tx_servo_pos.send(Timestamp::now(clock), cmd);

        Ok(StepResult::Continue)
    }
}
