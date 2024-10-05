pub mod localplotter;
pub mod plotter;

use rust_data_inspector::PlotSignalError;
use thiserror::Error;

use crate::telemetry::TelemetryError;

#[derive(Debug, Error)]
pub enum PlotterError {
    #[error("Error plotting channel")]
    PlotSignalError(#[from] PlotSignalError),

    #[error("Error subscribing to telemetry")]
    TelemetryError(#[from] TelemetryError),

    #[error("Channel was not registered")]
    UnregisteredChannel,

    #[error("Plotter closed")]
    Closed,
}
