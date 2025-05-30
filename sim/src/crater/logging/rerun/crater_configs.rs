use anyhow::Result;

use crater_gnc::{components::ada::AdaResult, datatypes::sensors::{ImuSensorSample, MagnetometerSensorSample}};
use rerun::RecordingStream;

use crate::crater::{
    aero::aerodynamics::AeroState,
    engine::engine::RocketEngineMassProperties,
    events::{GncEventItem, SimEvent},
    gnc::ServoPosition,
    rocket::{
        mass::RocketMassProperties,
        rocket_data::{RocketAccelerations, RocketActions, RocketState},
    },
};

use super::{
    crater_log_impl::{
        AdaOutputLog, AeroStateLog, GncEventLog, IMUSampleLog, MagnetometerSampleLog,
        RocketAccelLog, RocketActionsLog, RocketEngineMassPropertiesLog, RocketMassPropertiesLog,
        RocketStateRawLog, RocketStateUILog, ServoPositionLog, SimEventLog,
    },
    rerun_logger::{RerunLogConfig, RerunLoggerBuilder},
};

#[derive(Debug, Clone)]
pub struct CraterUiLogConfig;

impl RerunLogConfig for CraterUiLogConfig {
    fn init_rec(&self, rec: &mut RecordingStream) -> Result<()> {
        rec.log_static("/", &rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN())?;

        rec.set_duration_secs("sim_time", 0.0);

        rec.log(
            "rocket",
            &rerun::Asset3D::from_file_path("assets/sidewinder.obj")?,
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

        builder.log_telemetry::<AeroState>(
            "/rocket/aerostate",
            "timeseries/rocket/aerostate",
            AeroStateLog::default(),
        )?;
        builder.log_telemetry::<RocketActions>(
            "/rocket/actions",
            "timeseries/rocket/actions",
            RocketActionsLog::default(),
        )?;
        builder.log_telemetry::<RocketAccelerations>(
            "/rocket/accel",
            "timeseries/rocket/accel",
            RocketAccelLog::default(),
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
            "/rocket/mass/rocket",
            "timeseries/rocket/mass/rocket",
            RocketMassPropertiesLog::default(),
        )?;
        builder.log_telemetry::<RocketEngineMassProperties>(
            "/rocket/mass/engine",
            "timeseries/rocket/mass/engine",
            RocketEngineMassPropertiesLog::default(),
        )?;
        builder.log_telemetry::<ImuSensorSample>(
            "/sensors/ideal_imu/translated",
            "timeseries/sensors/ideal_imu/translated",
            IMUSampleLog::default(),
        )?;
        builder.log_telemetry::<ImuSensorSample>(
            "/sensors/ideal_imu/cg",
            "timeseries/sensors/ideal_imu/cg",
            IMUSampleLog::default(),
        )?;
        builder.log_telemetry::<MagnetometerSensorSample>(
            "/sensors/ideal_mag",
            "timeseries/sensors/ideal_mag",
            MagnetometerSampleLog::default(),
        )?;
        builder.log_telemetry_mp::<SimEvent>(
            "/sim/events",
            "log/sim/events",
            SimEventLog::default(),
        )?;
        builder.log_telemetry_mp::<GncEventItem>(
            "/gnc/events",
            "log/gnc/events",
            GncEventLog::default(),
        )?;
        builder.log_telemetry::<AdaResult>(
            "/gnc/fsw/ada",
            "timeseries/gnc/fsw/ada",
            AdaOutputLog::default(),
        )?;

        Ok(())
    }
}
