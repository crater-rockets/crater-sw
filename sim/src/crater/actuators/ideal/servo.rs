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

    last_pos: Option<ServoPosition>,
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
            last_pos: None,
        })
    }
}

impl Node for IdealServo {
    fn step(&mut self, _: usize, _: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
        while let Ok(Timestamped(_, cmd)) = self.rx_control.try_recv() {
            self.last_pos = Some(cmd);
        }

        self.tx_servo_pos.send(
            Timestamp::now(clock),
            self.last_pos
                .as_ref()
                .map(|v| v.clone())
                .unwrap_or(ServoPosition::default()),
        );

        Ok(StepResult::Continue)
    }
}
