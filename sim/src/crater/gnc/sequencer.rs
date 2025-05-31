use std::{
    any::{Any, TypeId},
    collections::HashMap,
    fs,
    path::Path,
};

use chrono::TimeDelta;
use crater_gnc::{events::GncEvent, mav_crater::ComponentId};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    core::time::{Clock, Timestamp},
    crater::events::{GncEventEnum, SimEvent},
    nodes::{Node, NodeContext, NodeTelemetry, StepResult},
    parameters,
    telemetry::{TelemetryError, TelemetrySender},
};

use super::{MixedServoPosition, ServoPosition};

#[derive(Debug, Error)]
pub enum Error {
    #[error("Channel {0} was already used with a different type")]
    IncosistentType(String),

    #[error("Telemetry error: {0}")]
    TelemetryError(#[from] TelemetryError),

    #[error("Not monotonic time sequence at index {index}, time {time}, channel '{channel}'")]
    TimeNotMonotonic {
        index: usize,
        time: f64,
        channel: String,
    },

    #[error("Error deserializing sequence json")]
    Deserialization(#[from] serde_json::Error),

    #[error("Error reading from file")]
    IoError(#[from] std::io::Error),

    #[error("Error parsing parameter")]
    Parameter(#[from] parameters::Error),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum Command {
    GncEvent(GncEventEnum),
    SimEvent(SimEvent),
    ServoPositionDeg { pos: [f64; 4] },
    MixedServoPositionDeg { pos: [f64; 4] },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SequenceEntry {
    time: f64,
    channel: String,
    cmd: Command,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum StartCondition {
    Sim,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Sequence {
    name: String,
    start: StartCondition,
    seq: Vec<SequenceEntry>,
}

pub struct Sequencer {
    sequences: Vec<(usize, Sequence)>,
    senders: HashMap<String, Box<dyn Any + Send>>,
}

impl Sequencer {
    pub fn from_files(json_files: &[&Path], ctx: NodeContext) -> Result<Self, Error> {
        let mut sequences = vec![];

        for json_file in json_files {
            let json_str = fs::read_to_string(json_file)?;
            let sequence = serde_json::from_str::<Sequence>(&json_str)?;
            sequences.push(sequence);
        }

        Self::new(sequences, ctx)
    }

    pub fn from_params(ctx: NodeContext) -> Result<Self, Error> {
        let sequences = ctx
            .parameters()
            .get_param("sim.rocket.sequencer.sequences")?
            .value_string_arr()?
            .to_owned();

        Self::from_files(
            sequences
                .iter()
                .map(|p| Path::new(p))
                .collect::<Vec<&Path>>()
                .as_slice(),
            ctx,
        )
    }

    pub fn new(sequences: Vec<Sequence>, ctx: NodeContext) -> Result<Self, Error> {
        let mut senders = HashMap::new();

        let mut last_time = 0.0;

        for sequence in sequences.iter() {
            for (i, entry) in sequence.seq.iter().enumerate() {
                if entry.time < last_time {
                    return Err(Error::TimeNotMonotonic {
                        index: i,
                        time: entry.time,
                        channel: entry.channel.clone(),
                    });
                }

                last_time = entry.time;
                Self::subscribe(&mut senders, entry, ctx.telemetry())?;
            }
        }

        Ok(Self {
            sequences: sequences.into_iter().map(|s| (0, s)).collect(),
            senders,
        })
    }

    fn add_sender<T: Clone + Send + 'static>(
        senders: &mut HashMap<String, Box<dyn Any + Send>>,
        channel: &String,
        tm: &NodeTelemetry,
        multi_prod: bool,
    ) -> Result<(), Error> {
        if let Some(sender) = senders.get(channel) {
            if (&**sender).type_id() == TypeId::of::<TelemetrySender<T>>() {
                Ok(())
            } else {
                Err(Error::IncosistentType(channel.to_string()))
            }
        } else {
            let sender = if multi_prod {
                tm.publish_mp::<T>(&channel)
            } else {
                tm.publish::<T>(&channel)
            }?;
            senders.insert(channel.to_string(), Box::new(sender));
            Ok(())
        }
    }

    fn subscribe(
        senders: &mut HashMap<String, Box<dyn Any + Send>>,
        entry: &SequenceEntry,
        tm: &NodeTelemetry,
    ) -> Result<(), Error> {
        match &entry.cmd {
            Command::GncEvent(_) => {
                Self::add_sender::<GncEvent>(senders, &entry.channel, tm, true)?;
            }
            Command::SimEvent(_) => {
                Self::add_sender::<SimEvent>(senders, &entry.channel, tm, true)?;
            }
            Command::ServoPositionDeg { .. } => {
                Self::add_sender::<ServoPosition>(senders, &entry.channel, tm, false)?;
            }
            Command::MixedServoPositionDeg { .. } => {
                Self::add_sender::<ServoPosition>(senders, &entry.channel, tm, false)?;
            }
        }

        Ok(())
    }

    fn execute(
        senders: &mut HashMap<String, Box<dyn Any + Send>>,
        ts: Timestamp,
        entry: &SequenceEntry,
    ) {
        let sender = senders
            .get(&entry.channel)
            .expect("Sequence entry with no matching sender!");

        match &entry.cmd {
            Command::SimEvent(ev) => {
                let sender = sender
                    .downcast_ref::<TelemetrySender<SimEvent>>()
                    .expect("Bad sender cast");
                sender.send(ts, ev.clone());
            }
            Command::GncEvent(ev) => {
                let sender = sender
                    .downcast_ref::<TelemetrySender<GncEvent>>()
                    .expect("Bad sender cast");

                sender.send(
                    ts,
                    GncEvent {
                        src: ComponentId::SimSequencer,
                        event: *ev,
                    },
                );
            }
            Command::ServoPositionDeg { pos } => {
                let sender = sender
                    .downcast_ref::<TelemetrySender<ServoPosition>>()
                    .expect("Bad sender cast");
                sender.send(ts, (*pos).map(|v| v.to_radians()).into());
            }
            Command::MixedServoPositionDeg { pos } => {
                let sender = sender
                    .downcast_ref::<TelemetrySender<ServoPosition>>()
                    .expect("Bad sender cast");

                let mixed: MixedServoPosition = (*pos).map(|v| v.to_radians()).into();
                sender.send(ts, mixed.unmix());
            }
        }
    }
}

impl Node for Sequencer {
    fn step(
        &mut self,
        _i: usize,
        _dt: TimeDelta,
        clock: &dyn Clock,
    ) -> anyhow::Result<crate::nodes::StepResult> {
        let t = clock.monotonic().elapsed_seconds_f64();

        for (next_step_index, sequence) in self.sequences.iter_mut() {
            while *next_step_index < sequence.seq.len() && sequence.seq[*next_step_index].time <= t
            {
                Self::execute(
                    &mut self.senders,
                    Timestamp::now(clock),
                    &sequence.seq[*next_step_index],
                );

                *next_step_index += 1;
            }
        }

        Ok(StepResult::Continue)
    }
}

#[cfg(test)]
mod tests {
    use super::{Sequence, SequenceEntry};

    #[test]
    fn test_ser() {
        let mut seq = Sequence {
            name: "Test".to_string(),
            start: super::StartCondition::Sim,
            seq: vec![],
        };

        seq.seq.push(SequenceEntry {
            time: 1.1,
            channel: "/a/b/c".to_string(),
            cmd: super::Command::GncEvent(crater_gnc::events::Event::CmdAdaCalibrate),
        });

        seq.seq.push(SequenceEntry {
            time: 1.2,
            channel: "/a/b/d".to_string(),
            cmd: super::Command::MixedServoPositionDeg {
                pos: [1.1, 2.2, 3.3, 4.4],
            },
        });

        let s = serde_json::to_string(&seq).unwrap();

        println!("{s}");
    }
}
