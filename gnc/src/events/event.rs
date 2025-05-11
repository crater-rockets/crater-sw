#[derive(Debug, Clone, Copy, PartialEq)]
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
