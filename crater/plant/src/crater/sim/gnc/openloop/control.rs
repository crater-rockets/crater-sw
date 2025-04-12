use std::fs;

use crate::{
    crater::sim::gnc::{datatypes::ServoPosition, MixedServoPosition},
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryDispatcher, TelemetrySender},
};
use anyhow::{Context, Result};
use chrono::TimeDelta;
use crater_core::time::{Clock, Timestamp};
use nalgebra::Vector4;
use rand::seq;
use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
struct ServoSequence {
    actuations: Vec<ServoActuation>,
}

#[derive(Debug, Clone, Deserialize)]
struct ServoActuation {
    time: f64,
    control_pos_mixed: [f64; 4],
}

#[derive(Debug)]
struct SequenceRunner {
    sequence: ServoSequence,
    last_index: usize,
}

impl SequenceRunner {
    fn new(mut sequence: ServoSequence) -> Self {
        if sequence.actuations.len() == 0 || sequence.actuations.get(0).unwrap().time > 0.0 {
            sequence.actuations.insert(
                0,
                ServoActuation {
                    time: 0.0,
                    control_pos_mixed: [0.0, 0.0, 0.0, 0.0],
                },
            );
        }

        for act in sequence.actuations.iter_mut() {
            for p in act.control_pos_mixed.iter_mut() {
                *p = p.to_radians();
            }
        }

        Self {
            sequence,
            last_index: 0,
        }
    }

    fn get_actuation(&mut self, t: f64) -> MixedServoPosition {
        let mut new_index = self.last_index;

        for act in self.sequence.actuations.iter().skip(self.last_index + 1) {
            if t >= act.time {
                new_index += 1;
            } else {
                break;
            }
        }

        self.last_index = new_index;

        self.sequence
            .actuations
            .get(new_index)
            .unwrap()
            .control_pos_mixed
            .into()
    }
}

#[derive(Debug)]
pub struct OpenloopControl {
    tx_servo_cmd: TelemetrySender<ServoPosition>,
    seq_runner: SequenceRunner,
}

impl OpenloopControl {
    pub fn new(ctx: NodeContext) -> Result<Self> {
        let tx_servo_cmd = ctx.telemetry().publish("/gnc/control/servo_command")?;

        let sequence_file = ctx
            .parameters()
            .get_string("/sim/rocket/crater/gnc/openloop/sequence")?;

        let sequence_string =
            fs::read_to_string(sequence_file.clone()).context(format!("path={sequence_file}"))?;
        let sequence: ServoSequence = toml::from_str(&sequence_string)?;
        let seq_runner = SequenceRunner::new(sequence);
        Ok(Self {
            tx_servo_cmd,
            seq_runner,
        })
    }
}

impl Node for OpenloopControl {
    fn step(&mut self, _: usize, _: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
        let t = Timestamp::now(clock).monotonic.elapsed_seconds_f64();

        let cmd = self.seq_runner.get_actuation(t);

        self.tx_servo_cmd.send(Timestamp::now(clock), cmd.unmix());

        Ok(StepResult::Continue)
    }
}
