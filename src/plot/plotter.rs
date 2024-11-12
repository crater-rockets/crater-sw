use std::collections::HashMap;

use prost_reflect::{DynamicMessage, FieldDescriptor, Kind, MessageDescriptor, Value};
use rust_data_inspector::{
    PlotSampleSender, PlotSignalError, PlotSignalSample, PlotSignalSendError, PlotSignals,
};

use crate::core::time::nsec_to_sec_f64;

use super::PlotterError;

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
    ) -> Result<(), PlotterError> {
        let field_senders = build_sender(signals, channel, desc)?;

        self.senders.insert(channel.to_string(), field_senders);

        Ok(())
    }

    pub fn plot(&mut self, channel: &str, ts: f64, msg: DynamicMessage) -> Result<(), PlotterError> {
        let senders = self
            .senders
            .get_mut(channel)
            .ok_or(PlotterError::UnregisteredChannel)?;

        plot_message(ts, &msg, senders).map_err(|_| PlotterError::Closed)?;

        Ok(())
    }
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

fn plot_message(
    time: f64,
    msg: &DynamicMessage,
    field_senders: &mut HashMap<u32, FieldSender>,
) -> Result<(), PlotSignalSendError<PlotSignalSample>> {
    for (desc, val) in msg.fields() {
        let res = match (
            val,
            field_senders
                .get_mut(&desc.number())
                .expect("Sender for provided field number not found!"),
        ) {
            (Value::Bool(v), FieldSender::Sender(s)) => s.send(PlotSignalSample {
                time,
                value: (*v as u8) as f64,
            }),
            (Value::I32(v), FieldSender::Sender(s)) => s.send(PlotSignalSample {
                time,
                value: *v as f64,
            }),
            (Value::I64(v), FieldSender::Sender(s)) => s.send(PlotSignalSample {
                time,
                value: *v as f64,
            }),
            (Value::U32(v), FieldSender::Sender(s)) => s.send(PlotSignalSample {
                time,
                value: *v as f64,
            }),
            (Value::U64(v), FieldSender::Sender(s)) => s.send(PlotSignalSample {
                time,
                value: *v as f64,
            }),
            (Value::F32(v), FieldSender::Sender(s)) => s.send(PlotSignalSample {
                time,
                value: *v as f64,
            }),
            (Value::F64(v), FieldSender::Sender(s)) => s.send(PlotSignalSample {
                time,
                value: *v as f64,
            }),
            (Value::EnumNumber(v), FieldSender::Sender(s)) => s.send(PlotSignalSample {
                time,
                value: *v as f64,
            }),
            (Value::Message(msg), FieldSender::Nested(map)) => plot_message(time, msg, map),
            (Value::String(_), _) => {
                // Ignore
                Ok(())
            }
            (Value::Bytes(_), _) => {
                // Ignore
                Ok(())
            }
            (Value::List(_), _) => {
                // Ignore
                Ok(())
            }
            (Value::Map(_), _) => {
                // Ignore
                Ok(())
            }
            _ => {
                panic!("Unexpected field sender type");
            }
        };

        if let Err(e) = res {
            return Err(e);
        }
    }
    Ok(())
}
