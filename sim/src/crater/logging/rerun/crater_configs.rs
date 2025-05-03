use anyhow::Result;

use rerun::RecordingStream;

use crate::crater::sim::{
    gnc::ServoPosition,
    rocket_data::{AeroAngles, RocketActions, RocketMassProperties, RocketState}, sensors::{IMUSample, MagnetometerSample},
};

use super::{
    crater_log_impl::{
        AeroAnglesLog, IMUSampleLog, MagnetometerSampleLog, RocketActionsLog, RocketMassPropertiesLog, RocketStateRawLog, RocketStateUILog, ServoPositionLog
    },
    rerun_logger::{RerunLogConfig, RerunLoggerBuilder},
};

#[derive(Debug, Clone)]
pub struct CraterUiLogConfig;

impl RerunLogConfig for CraterUiLogConfig {
    fn init_rec(&self, rec: &mut RecordingStream) -> Result<()> {
        rec.log_static("/", &rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN)?;

        rec.set_time_seconds("sim_time", 0.0);

        rec.log(
            "rocket",
            &rerun::Asset3D::from_file("assets/sidewinder.obj")?,
        )?;

        Ok(())
    }

    fn subscribe_telem(&self, builder: &mut RerunLoggerBuilder) -> Result<()> {
        builder.log_telemetry::<RocketState>(
            "/rocket/state",
            "timeseries/rocket/state",
            RocketStateRawLog::default(),
        )?;
        builder.log_telemetry::<RocketState>(
            "/rocket/state",
            "timeseries/rocket/state",
            RocketStateUILog::default(),
        )?;

        builder.log_telemetry::<AeroAngles>(
            "/rocket/aero_angles",
            "timeseries/rocket/aero_angles",
            AeroAnglesLog::default(),
        )?;
        builder.log_telemetry::<RocketActions>(
            "/rocket/actions",
            "timeseries/rocket/actions",
            RocketActionsLog::default(),
        )?;
        builder.log_telemetry::<ServoPosition>(
            "/gnc/control/servo_command",
            "timeseries/gnc/control/servo_command",
            ServoPositionLog::default(),
        )?;
        builder.log_telemetry::<ServoPosition>(
            "/actuators/servo_position",
            "timeseries/actuators/servo_position",
            ServoPositionLog::default(),
        )?;
        builder.log_telemetry::<RocketMassProperties>(
            "/rocket/masses",
            "timeseries/rocket/masses",
            RocketMassPropertiesLog::default(),
        )?;
        builder.log_telemetry::<IMUSample>(
            "/sensors/ideal_imu/translated",
            "timeseries/sensors/ideal_imu/translated",
            IMUSampleLog::default(),
        )?;
        builder.log_telemetry::<IMUSample>(
            "/sensors/ideal_imu/cg",
            "timeseries/sensors/ideal_imu/cg",
            IMUSampleLog::default(),
        )?;
        builder.log_telemetry::<MagnetometerSample>(
            "/sensors/ideal_mag",
            "timeseries/sensors/ideal_mag",
            MagnetometerSampleLog::default(),
        )?;
        Ok(())
    }
}
