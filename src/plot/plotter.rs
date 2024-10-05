use std::collections::HashMap;

use anyhow::{anyhow, Result};
use prost_reflect::{DynamicMessage, FieldDescriptor, Kind, MessageDescriptor, Value};
use rust_data_inspector::{
    DataInspector, PlotSampleSender, PlotSignalError, PlotSignalSample, PlotSignals,
};

use crate::utils::time::nsec_to_sec_f64;

enum FieldSender {
    Sender(PlotSampleSender),
    Nested(HashMap<u32, FieldSender>),
}

pub struct Plotter {
    senders: HashMap<String, HashMap<u32, FieldSender>>,
}

impl Plotter {
    pub fn new() -> Self {
        Plotter {
            senders: HashMap::new(),
        }
    }

    pub fn register(
        &mut self,
        signals: &mut PlotSignals,
        channel: &str,
        desc: MessageDescriptor,
    ) -> Result<()> {
        let field_senders = build_sender(signals, channel, desc)?;

        self.senders.insert(channel.to_string(), field_senders);

        Ok(())
    }

    pub fn plot(&mut self, channel: &str, msg: DynamicMessage) -> Result<()> {
        let senders = self
            .senders
            .get_mut(channel)
            .ok_or(anyhow!("Channel not registered"))?;

        let ts = nsec_to_sec_f64(
            msg.get_field_by_name("timestamp")
                .unwrap()
                .as_i64()
                .unwrap(),
        );

        plot_message(ts, &msg, senders);

        Ok(())
    }

    // fn run(self) -> Result<()> {
    //     DataInspector::run_native("plotter", self.signals).unwrap();
    //     Ok(())
    // }
}

fn build_sender(
    signals: &mut PlotSignals,
    channel: &str,
    desc: MessageDescriptor,
) -> Result<HashMap<u32, FieldSender>, PlotSignalError> {
    fn make_sender_rec(
        signals: &mut PlotSignals,
        field: &FieldDescriptor,
        name: &str,
        field_senders: &mut HashMap<u32, FieldSender>,
    ) -> Result<(), PlotSignalError> {
        let name = format!("{}/{}", name, field.name());

        // TODO: Repeated fields are currently overly simplified by assuming they only have one element

        match field.kind() {
            Kind::Message(m) => {
                let mut nested = HashMap::new();
                for field in m.fields() {
                    make_sender_rec(signals, &field, name.as_str(), &mut nested)?;
                }
                field_senders.insert(field.number(), FieldSender::Nested(nested));
            }
            _ => {
                let (_, sender) = signals.add_signal(name.as_str())?;
                field_senders.insert(field.number(), FieldSender::Sender(sender));
            }
        }

        Ok(())
    }

    let mut field_senders = HashMap::new();

    for field in desc.fields() {
        make_sender_rec(signals, &field, channel, &mut field_senders)?;
    }

    Ok(field_senders)
}

fn plot_message(time: f64, msg: &DynamicMessage, field_senders: &mut HashMap<u32, FieldSender>) {
    for (desc, val) in msg.fields() {
        match (val, field_senders.get_mut(&desc.number()).unwrap()) {
            (Value::Bool(v), FieldSender::Sender(s)) => {
                s.send(PlotSignalSample {
                    time,
                    value: (*v as u8) as f64,
                })
                .unwrap();
            }
            (Value::I32(v), FieldSender::Sender(s)) => {
                s.send(PlotSignalSample {
                    time,
                    value: *v as f64,
                })
                .unwrap();
            }
            (Value::I64(v), FieldSender::Sender(s)) => {
                s.send(PlotSignalSample {
                    time,
                    value: *v as f64,
                })
                .unwrap();
            }
            (Value::U32(v), FieldSender::Sender(s)) => {
                s.send(PlotSignalSample {
                    time,
                    value: *v as f64,
                })
                .unwrap();
            }
            (Value::U64(v), FieldSender::Sender(s)) => {
                s.send(PlotSignalSample {
                    time,
                    value: *v as f64,
                })
                .unwrap();
            }
            (Value::F32(v), FieldSender::Sender(s)) => {
                s.send(PlotSignalSample {
                    time,
                    value: *v as f64,
                })
                .unwrap();
            }
            (Value::F64(v), FieldSender::Sender(s)) => {
                s.send(PlotSignalSample {
                    time,
                    value: *v as f64,
                })
                .unwrap();
            }
            (Value::EnumNumber(v), FieldSender::Sender(s)) => {
                s.send(PlotSignalSample {
                    time,
                    value: *v as f64,
                })
                .unwrap();
            }
            (Value::Message(msg), FieldSender::Nested(map)) => {
                plot_message(time, msg, map);
            }
            (Value::String(_), _) => {
                // Ignore
            }
            (Value::Bytes(_), _) => {
                // Ignore
            }
            (Value::List(_), _) => {
                // Ignore
            }
            (Value::Map(_), _) => {
                // Ignore
            }
            _ => {
                panic!("Unexpected field sender type");
            }
        }
    }
}
