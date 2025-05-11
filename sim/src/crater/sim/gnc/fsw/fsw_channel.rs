use chrono::TimeDelta;
use crater_gnc::{
    InstantU64,
    common::Ts,
    hal::channel::{Full, Receiver, Sender},
};

use crate::{
    core::time::Timestamp,
    telemetry::{TelemetryReceiver, TelemetrySender},
};

impl<T: 'static + Clone> Sender<T> for TelemetrySender<T> {
    fn try_send(&mut self, ts: crater_gnc::Instant, item: T) -> Result<(), Full<T>> {
        self.send(
            Timestamp {
                utc: None,
                monotonic: TimeDelta::microseconds(ts.0.duration_since_epoch().to_micros() as i64)
                    .into(),
            },
            item,
        );

        Ok(())
    }
}

impl<T: 'static + Clone> Receiver<T> for TelemetryReceiver<T> {
    fn try_recv(&mut self) -> Option<Ts<T>> {
        if let Ok(v) = TelemetryReceiver::try_recv(&self) {
            Some(Ts::<T> {
                t: InstantU64::from_ticks(
                    v.0.monotonic.elapsed().num_microseconds().unwrap() as u64
                )
                .into(),
                v: v.1,
            })
        } else {
            None
        }
    }

    fn try_recv_last(&mut self) -> Option<Ts<T>> {
        let mut out = None;

        while let Some(v) = self.try_recv() {
            out = Some(v);
        }

        out
    }

    fn capacity(&self) -> usize {
        todo!()
    }

    fn is_empty(&self) -> bool {
        todo!()
    }

    fn is_full(&self) -> bool {
        todo!()
    }

    fn len(&self) -> usize {
        todo!()
    }

    fn num_lagged(&self) -> usize {
        todo!()
    }
}
