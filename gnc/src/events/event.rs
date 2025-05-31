use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Event {
    Step,

    Meco,
    
    // Flight State Transitions
    FlightStateReady,
    FlightLiftoff,

    // Fmm
    CmdFmmCalibrate,
    CmdFmmArm,
    CmdFmmForceLiftoff,

    // Ada
    AdaCalibrationDone,

    CmdAdaCalibrate,
}
