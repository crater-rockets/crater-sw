use anyhow::Result;

use crater_gnc::{
    components::ada::AdaResult,
    datatypes::{
        gnc::NavigationOutput,
        sensors::{ImuSensorSample, MagnetometerSensorSample},
    },
};
use rerun::RecordingStream;

use crate::crater::{
    aero::aerodynamics::AeroState,
    channels,
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
        NavigationOutputLog, RocketAccelLog, RocketActionsLog, RocketEngineMassPropertiesLog,
        RocketMassPropertiesLog, RocketStateRawLog, RocketStateUILog, ServoPositionLog,
        SimEventLog,
    },
    rerun_logger::{ChannelName, RerunLogConfig, RerunLoggerBuilder},
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
            ChannelName::from_base_path(channels::rocket::STATE, "timeseries"),
            RocketStateRawLog::default(),
        )?;
        builder.log_telemetry::<RocketState>(
            ChannelName::from_base_path(channels::rocket::STATE, "timeseries"),
            RocketStateUILog::default(),
        )?;

        builder.log_telemetry::<AeroState>(
            ChannelName::from_base_path(channels::rocket::AERO_STATE, "timeseries"),
            AeroStateLog::default(),
        )?;
        builder.log_telemetry::<RocketActions>(
            ChannelName::from_base_path(channels::rocket::ACTIONS, "timeseries"),
            RocketActionsLog::default(),
        )?;
        builder.log_telemetry::<RocketAccelerations>(
            ChannelName::from_base_path(channels::rocket::ACCEL, "timeseries"),
            RocketAccelLog::default(),
        )?;
        builder.log_telemetry::<ServoPosition>(
            ChannelName::from_base_path(channels::gnc::SERVO_COMMAND, "timeseries"),
            ServoPositionLog::default(),
        )?;
        builder.log_telemetry::<ServoPosition>(
            ChannelName::from_base_path(channels::actuators::IDEAL_SERVO_POSITION, "timeseries"),
            ServoPositionLog::default(),
        )?;
        builder.log_telemetry::<RocketMassProperties>(
            ChannelName::from_base_path(channels::rocket::MASS_ROCKET, "timeseries"),
            RocketMassPropertiesLog::default(),
        )?;
        builder.log_telemetry::<RocketEngineMassProperties>(
            ChannelName::from_base_path(channels::rocket::MASS_ENGINE, "timeseries"),
            RocketEngineMassPropertiesLog::default(),
        )?;
        builder.log_telemetry::<ImuSensorSample>(
            ChannelName::from_base_path(channels::sensors::IDEAL_IMU, "timeseries"),
            IMUSampleLog::default(),
        )?;
        builder.log_telemetry::<ImuSensorSample>(
            ChannelName::from_base_path(channels::sensors::IDEAL_IMU_CG, "timeseries"),
            IMUSampleLog::default(),
        )?;
        builder.log_telemetry::<ImuSensorSample>(
            ChannelName::from_base_path(channels::sensors::IMU_REAL, "timeseries"),
            IMUSampleLog::default(),
        )?;
        builder.log_telemetry::<MagnetometerSensorSample>(
            ChannelName::from_base_path(channels::sensors::IDEAL_MAGNETOMETER, "timeseries"),
            MagnetometerSampleLog::default(),
        )?;
        builder.log_telemetry_mp::<SimEvent>(
            ChannelName::from_base_path(channels::sim::SIM_EVENTS, "log"),
            SimEventLog::default(),
        )?;
        builder.log_telemetry_mp::<GncEventItem>(
            ChannelName::from_base_path(channels::gnc::GNC_EVENTS, "log"),
            GncEventLog::default(),
        )?;
        builder.log_telemetry::<AdaResult>(
            ChannelName::from_base_path(channels::gnc::ADA_OUTPUT, "timeseries"),
            AdaOutputLog::default(),
        )?;
        builder.log_telemetry::<NavigationOutput>(
            ChannelName::from_base_path(channels::sensors::IDEAL_NAV_OUTPUT, "timeseries"),
            NavigationOutputLog::default(),
        )?;
        builder.log_telemetry::<NavigationOutput>(
            ChannelName::from_base_path(channels::gnc::NAV_OUTPUT, "timeseries"),
            NavigationOutputLog::default(),
        )?;
        Ok(())
    }
}
