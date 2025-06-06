use crater_gnc::{
    components::ada::AdaResult,
    datatypes::{
        gnc::NavigationOutput,
        sensors::{ImuSensorSample, MagnetometerSensorSample, PressureSensorSample},
    },
};
use map_3d::ned2geodetic;
use nalgebra::{Matrix3, RealField, SMatrix, UnitQuaternion, Vector3, Vector4};
use num_traits::{AsPrimitive, Float};
use rerun::{
    Quaternion, RecordingStream, TensorData, TextLogLevel, components::RotationQuat,
    external::arrow::buffer::ScalarBuffer,
};

use crate::{
    core::time::Timestamp,
    crater::{
        aero::aerodynamics::AeroState,
        engine::engine::RocketEngineMassProperties,
        events::{GncEventItem, SimEvent},
        gnc::ServoPosition,
        rocket::{
            mass::RocketMassProperties,
            rocket_data::{RocketAccelerations, RocketActions, RocketState},
        },
    },
};

use super::rerun_logger::RerunWrite;
use anyhow::{Ok, Result};

#[derive(Default)]
pub struct RocketStateRawLog;

impl RerunWrite for RocketStateRawLog {
    type Telem = RocketState;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        timeline: &str,
        ent_path: &str,
        ts: Timestamp,
        state: RocketState,
    ) -> Result<()> {
        let ts_seconds = ts.monotonic.elapsed_seconds_f64();
        rec.set_duration_secs(timeline, ts_seconds);

        log_vector3_timeseries(rec, format!("{ent_path}/pos_n"), &state.pos_n_m())?;
        log_vector3_timeseries(rec, format!("{ent_path}/vel_n"), &state.vel_n_m_s())?;
        log_vector3_timeseries(
            rec,
            format!("{ent_path}/ang_vel_b"),
            &state.angvel_b_rad_s(),
        )?;
        log_quat_timeseries(
            rec,
            &state.quat_nb(),
            format!("{ent_path}/orient/quat"),
            format!("{ent_path}/orient/euler"),
        )?;

        Ok(())
    }
}

#[derive(Default)]
pub struct RocketStateUILog {
    trajectory_ned_3d: Vec<[f32; 3]>,
    trajectory_geodetic: Vec<[f64; 2]>,
    ts_last_element: f64,
}

impl RerunWrite for RocketStateUILog {
    type Telem = RocketState;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        timeline: &str,
        ent_path: &str,
        ts: Timestamp,
        state: RocketState,
    ) -> Result<()> {
        let origin = [
            f64::to_radians(41.8080239),
            f64::to_radians(14.0548082),
            1411.211,
        ];
        let pos = state.pos_n_m();
        let pos_f32_arr: [f32; 3] = pos.map(|v| v as f32).into();

        let (lat, lon, _) = ned2geodetic(
            pos[0],
            pos[1],
            pos[2],
            origin[0],
            origin[1],
            origin[2],
            map_3d::Ellipsoid::WGS84,
        );

        let ts_seconds = ts.monotonic.elapsed_seconds_f64();
        rec.set_duration_secs(timeline, ts_seconds);

        rec.log(
            "/frame/body_centered",
            &rerun::Transform3D::from_translation(pos_f32_arr),
        )?;

        // Velocity body frame
        let vel_b = state.vel_b_m_s(&state.quat_nb());
        let vnorm = vel_b.norm();

        log_vector3_timeseries(rec, format!("{ent_path}/vel_b"), &vel_b)?;

        rec.log(
            format!("{ent_path}/vel_norm"),
            &rerun::Scalars::single(vnorm),
        )?;

        // Log trajectory less frequently, as every log contains the full position history, making logs huge
        if self.ts_last_element == 0.0 || ts_seconds - self.ts_last_element >= 0.1 {
            self.ts_last_element = ts_seconds;

            // Keep history of 3D position to display a 3D trajectory in Rerun
            self.trajectory_ned_3d
                .push([pos[0] as f32, pos[1] as f32, pos[2] as f32]);

            // Keep history of latitude and longitude to display trajectory over a map
            self.trajectory_geodetic
                .push([lat.to_degrees(), lon.to_degrees()]);

            // Trajectory lines
            rec.log(
                "trajectory/ned_3d",
                &rerun::LineStrips3D::new([self.trajectory_ned_3d.as_slice()]),
            )?;

            rec.log(
                "trajectory/geodetic",
                &rerun::GeoLineStrings::from_lat_lon([self.trajectory_geodetic.as_slice()]),
            )?;
        }

        // Current location point for map plot
        rec.log(
            "objects/position_geodetic",
            &rerun::GeoPoints::from_lat_lon([(lat, lon)])
                .with_radii([rerun::Radius::new_ui_points(10.0)])
                .with_colors([rerun::Color::from_rgb(255, 0, 0)]),
        )?;

        // Velocity vector
        let arrow_vec: [f32; 3] = (state.vel_b_m_s(&state.quat_nb()) / 10.0)
            .map(|v| v as f32)
            .into();

        rec.log(
            "objects/vectors/velocity",
            &rerun::Arrows3D::from_vectors([arrow_vec])
                .with_colors([rerun::Color::from_rgb(0, 255, 0)])
                .with_origins([[0.0, 0.0, 0.0]]),
        )?;

        // Transform
        let quat = state.quat_nb();
        let body_transform = rerun::Transform3D::from_translation_rotation(
            pos_f32_arr,
            rerun::Rotation3D::Quaternion(RotationQuat(Quaternion([
                quat.i as f32,
                quat.j as f32,
                quat.k as f32,
                quat.w as f32,
            ]))),
        );

        rec.log("rocket", &body_transform)?;
        rec.log("objects/vectors", &body_transform)?;

        Ok(())
    }
}

#[derive(Default)]
pub struct AeroStateLog;

impl RerunWrite for AeroStateLog {
    type Telem = AeroState;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        timeline: &str,
        ent_path: &str,
        ts: Timestamp,
        state: AeroState,
    ) -> Result<()> {
        rec.set_duration_secs(timeline, ts.monotonic.elapsed_seconds_f64());

        rec.log(
            format!("{ent_path}/alpha_deg"),
            &rerun::Scalars::single(state.angles.alpha_rad.to_degrees()),
        )?;

        rec.log(
            format!("{ent_path}/beta_deg"),
            &rerun::Scalars::single(state.angles.beta_rad.to_degrees()),
        )?;

        rec.log(
            format!("{ent_path}/beta_tan_deg"),
            &rerun::Scalars::single(state.angles.beta_tan_rad.to_degrees()),
        )?;

        rec.log(
            format!("{ent_path}/mach"),
            &rerun::Scalars::single(state.mach),
        )?;

        rec.log(
            format!("{ent_path}/air_density_kg_m3"),
            &rerun::Scalars::single(state.air_density_kg_m3),
        )?;

        rec.log(
            format!("{ent_path}/altitude_m"),
            &rerun::Scalars::single(state.altitude_m),
        )?;

        rec.log(
            format!("{ent_path}/v_air_norm_m_s"),
            &rerun::Scalars::single(state.v_air_norm_m_s),
        )?;

        log_vector3_timeseries(rec, format!("{ent_path}/v_air_b_m_s"), &state.v_air_b_m_s)?;
        log_vector3_timeseries(rec, format!("{ent_path}/w_b_rad_s"), &state.w_b_rad_s)?;

        Ok(())
    }
}

#[derive(Default)]
pub struct RocketActionsLog;

impl RerunWrite for RocketActionsLog {
    type Telem = RocketActions;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        timeline: &str,
        ent_path: &str,
        ts: Timestamp,
        actions: RocketActions,
    ) -> Result<()> {
        rec.set_duration_secs(timeline, ts.monotonic.elapsed_seconds_f64());

        log_vector3_timeseries(rec, format!("{ent_path}/thrust_b_n"), &actions.thrust_b_n)?;
        log_vector3_timeseries(
            rec,
            format!("{ent_path}/aero_force_b_n"),
            &actions.aero_actions.forces_b_n,
        )?;
        log_vector3_timeseries(
            rec,
            format!("{ent_path}/aero_moments_b_nm"),
            &actions.aero_actions.moments_b_nm,
        )?;

        let thrust_scaled: [f32; 3] = (actions.thrust_b_n / 20.0).map(|v| v as f32).into();
        let aero_force_scaled: [f32; 3] = (actions.aero_actions.forces_b_n / 1.0)
            .map(|v| v as f32)
            .into();

        rec.log(
            "objects/vectors/thurst",
            &rerun::Arrows3D::from_vectors([thrust_scaled])
                .with_colors([rerun::Color::from_rgb(255, 0, 0)])
                .with_origins([[2.0, 0.0, 0.0]]),
        )?;

        rec.log(
            "objects/vectors/aero_forces",
            &rerun::Arrows3D::from_vectors([aero_force_scaled])
                .with_colors([rerun::Color::from_rgb(0, 0, 255)])
                .with_origins([[0.0, 0.0, 0.0]]),
        )?;

        Ok(())
    }
}

#[derive(Default)]
pub struct RocketAccelLog;

impl RerunWrite for RocketAccelLog {
    type Telem = RocketAccelerations;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        timeline: &str,
        ent_path: &str,
        ts: Timestamp,
        accel: RocketAccelerations,
    ) -> Result<()> {
        rec.set_duration_secs(timeline, ts.monotonic.elapsed_seconds_f64());

        log_vector3_timeseries(rec, format!("{ent_path}/acc_b"), &accel.acc_b_m_s2)?;
        log_vector3_timeseries(rec, format!("{ent_path}/acc_n"), &accel.acc_n_m_s2)?;

        Ok(())
    }
}

#[derive(Default)]
pub struct ServoPositionLog;

impl RerunWrite for ServoPositionLog {
    type Telem = ServoPosition;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        timeline: &str,
        ent_path: &str,
        ts: Timestamp,
        servo_pos: ServoPosition,
    ) -> Result<()> {
        rec.set_duration_secs(timeline, ts.monotonic.elapsed_seconds_f64());

        log_matrix_timeseries(
            rec,
            format!("{ent_path}/raw"),
            &servo_pos.pos_rad.map(|x| x.to_degrees()),
            None,
            None,
        )?;

        let mixed = servo_pos.mix();
        log_matrix_timeseries(
            rec,
            format!("{ent_path}/mixed"),
            &mixed.pos_rad.map(|x| x.to_degrees()),
            Some(&[
                "yaw".to_string(),
                "pitch".to_string(),
                "roll".to_string(),
                "squeeze".to_string(),
            ]),
            None,
        )?;

        Ok(())
    }
}

#[derive(Default)]
pub struct RocketMassPropertiesLog;

impl RerunWrite for RocketMassPropertiesLog {
    type Telem = RocketMassProperties;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        timeline: &str,
        ent_path: &str,
        ts: Timestamp,
        mass: RocketMassProperties,
    ) -> Result<()> {
        rec.set_duration_secs(timeline, ts.monotonic.elapsed_seconds_f64());

        log_vector3_timeseries(rec, format!("{ent_path}/xcg_tot_m"), &mass.xcg_total_m)?;

        rec.log(
            format!("{ent_path}/mass_tot_kg"),
            &rerun::Scalars::single(mass.mass_kg),
        )?;

        rec.log(
            format!("{ent_path}/mass_dot_kg_s"),
            &rerun::Scalars::single(mass.mass_dot_kg_s),
        )?;

        log_matrix3_timeseries(rec, format!("{ent_path}/inertia_kgm2"), &mass.inertia_kgm2)?;
        log_matrix3_timeseries(
            rec,
            format!("{ent_path}/inertia_dot_kgm2_s"),
            &mass.inertia_dot_kgm2_s,
        )?;

        Ok(())
    }
}

#[derive(Default)]
pub struct RocketEngineMassPropertiesLog;

impl RerunWrite for RocketEngineMassPropertiesLog {
    type Telem = RocketEngineMassProperties;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        timeline: &str,
        ent_path: &str,
        ts: Timestamp,
        mass: RocketEngineMassProperties,
    ) -> Result<()> {
        rec.set_duration_secs(timeline, ts.monotonic.elapsed_seconds_f64());

        rec.log(
            format!("{ent_path}/xcg_eng_frame_m"),
            &rerun::Scalars::single(mass.xcg_eng_frame_m),
        )?;

        rec.log(
            format!("{ent_path}/xcg_dot_eng_frame_m"),
            &rerun::Scalars::single(mass.xcg_dot_eng_frame_m),
        )?;

        rec.log(
            format!("{ent_path}/mass_kg"),
            &rerun::Scalars::single(mass.mass_kg),
        )?;

        rec.log(
            format!("{ent_path}/mass_dot_kg_s"),
            &rerun::Scalars::single(mass.mass_dot_kg_s),
        )?;

        log_matrix3_timeseries(
            rec,
            format!("{ent_path}/inertia_eng_frame_kgm2"),
            &mass.inertia_eng_frame_kgm2,
        )?;
        log_matrix3_timeseries(
            rec,
            format!("{ent_path}/inertia_dot_eng_frame_kgm2"),
            &mass.inertia_dot_eng_frame_kgm2,
        )?;

        Ok(())
    }
}

#[derive(Default)]
pub struct IMUSampleLog;

impl RerunWrite for IMUSampleLog {
    type Telem = ImuSensorSample;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        timeline: &str,
        ent_path: &str,
        ts: Timestamp,
        imu: ImuSensorSample,
    ) -> Result<()> {
        rec.set_duration_secs(timeline, ts.monotonic.elapsed_seconds_f64());

        log_vector3_timeseries(rec, format!("{ent_path}/acc_m_s2"), &imu.accel_m_s2)?;

        let gyro_deg = imu.angvel_rad_s.map(|x| x.to_degrees());
        log_vector3_timeseries(rec, format!("{ent_path}/gyro_deg_s"), &gyro_deg)?;

        Ok(())
    }
}

#[derive(Default)]
pub struct MagnetometerSampleLog;

impl RerunWrite for MagnetometerSampleLog {
    type Telem = MagnetometerSensorSample;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        timeline: &str,
        ent_path: &str,
        ts: Timestamp,
        mag: MagnetometerSensorSample,
    ) -> Result<()> {
        rec.set_duration_secs(timeline, ts.monotonic.elapsed_seconds_f64());

        log_vector3_timeseries(rec, ent_path.to_string(), &mag.mag_field_b_gauss)?;

        Ok(())
    }
}

#[derive(Default)]
pub struct GncEventLog;

impl RerunWrite for GncEventLog {
    type Telem = GncEventItem;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        timeline: &str,
        ent_path: &str,
        ts: Timestamp,
        event: GncEventItem,
    ) -> Result<()> {
        rec.set_duration_secs(timeline, ts.monotonic.elapsed_seconds_f64());

        rec.log(
            format!("{}", ent_path),
            &rerun::TextLog::new(format!("{:?} from {:#?}", event.event, event.src))
                .with_level(TextLogLevel::INFO),
        )?;

        Ok(())
    }
}

#[derive(Default)]
pub struct SimEventLog;

impl RerunWrite for SimEventLog {
    type Telem = SimEvent;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        timeline: &str,
        ent_path: &str,
        ts: Timestamp,
        event: SimEvent,
    ) -> Result<()> {
        rec.set_duration_secs(timeline, ts.monotonic.elapsed_seconds_f64());

        rec.log(
            format!("{}", ent_path),
            &rerun::TextLog::new(format!("{:?}", event)).with_level(TextLogLevel::TRACE),
        )?;

        Ok(())
    }
}

#[derive(Default)]
pub struct AdaOutputLog;

impl RerunWrite for AdaOutputLog {
    type Telem = AdaResult;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        timeline: &str,
        ent_path: &str,
        ts: Timestamp,
        ada: AdaResult,
    ) -> Result<()> {
        rec.set_duration_secs(timeline, ts.monotonic.elapsed_seconds_f64());

        rec.log(
            format!("{}/altitude_m", ent_path),
            &rerun::Scalars::single(ada.altitude_m as f64),
        )?;

        rec.log(
            format!("{}/vertical_speed_m_s", ent_path),
            &rerun::Scalars::single(ada.vertical_speed_m_s as f64),
        )?;

        Ok(())
    }
}

#[derive(Default)]
pub struct PressureSensorSampleLog;

impl RerunWrite for PressureSensorSampleLog {
    type Telem = PressureSensorSample;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        timeline: &str,
        ent_path: &str,
        ts: Timestamp,
        data: Self::Telem,
    ) -> Result<()> {
        rec.set_duration_secs(timeline, ts.monotonic.elapsed_seconds_f64());

        rec.log(
            format!("{}/pressure_pa", ent_path),
            &rerun::Scalars::single(data.pressure_pa as f64),
        )?;

        if let Some(temp) = data.temperature_degc {
            rec.log(
                format!("{}/temperature_degc", ent_path),
                &rerun::Scalars::single(temp as f64),
            )?;
        }

        Ok(())
    }
}

#[derive(Default)]
pub struct ImuSensorSampleLog;

impl RerunWrite for ImuSensorSampleLog {
    type Telem = ImuSensorSample;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        timeline: &str,
        ent_path: &str,
        ts: Timestamp,
        data: Self::Telem,
    ) -> Result<()> {
        rec.set_duration_secs(timeline, ts.monotonic.elapsed_seconds_f64());

        log_vector3_timeseries(rec, format!("{}/accel_m_s2", ent_path), &data.accel_m_s2)?;
        log_vector3_timeseries(
            rec,
            format!("{}/ang_vel_deg_s", ent_path),
            &data.angvel_rad_s.map(|v| v.to_degrees()),
        )?;

        if let Some(temp) = data.temperature_degc {
            rec.log(
                format!("{}/temperature_degc", ent_path),
                &rerun::Scalars::single(temp as f64),
            )?;
        }

        rec.log(
            format!("{}/int_latency_s", ent_path),
            &rerun::Scalars::single(data.int_latency.0.to_micros() as f64 / 1_000_000f64),
        )?;

        rec.log(
            format!("{}/overrun_count", ent_path),
            &rerun::Scalars::single(data.overrun_count as f64),
        )?;

        Ok(())
    }
}

#[derive(Default)]
pub struct NavigationOutputLog;

impl RerunWrite for NavigationOutputLog {
    type Telem = NavigationOutput;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        timeline: &str,
        ent_path: &str,
        ts: Timestamp,
        data: Self::Telem,
    ) -> Result<()> {
        rec.set_duration_secs(timeline, ts.monotonic.elapsed_seconds_f64());

        log_quat_timeseries::<f32>(
            rec,
            &data.quat_nb,
            format!("{}/quat", ent_path),
            format!("{}/euler", ent_path),
        )?;

        log_vector3_timeseries(rec, format!("{}/pos_n_m", ent_path), &data.pos_n_m)?;
        log_vector3_timeseries(rec, format!("{}/vel_n_m_s", ent_path), &data.vel_n_m_s)?;
        log_vector3_timeseries(
            rec,
            format!("{}/angvel_unbias_b_rad_s", ent_path),
            &data.angvel_unbias_b_rad_s,
        )?;
        log_vector3_timeseries(
            rec,
            format!("{}/acc_unbias_b_m_s2", ent_path),
            &data.acc_unbias_b_m_s2,
        )?;

        Ok(())
    }
}

fn log_matrix_timeseries<T: Float + AsPrimitive<f64>, const R: usize, const C: usize>(
    rec: &mut RecordingStream,
    ent_path: String,
    matrix: &SMatrix<T, R, C>,
    row_names: Option<&[String]>,
    col_names: Option<&[String]>,
) -> Result<()> {
    for r in 0..R {
        for c in 0..C {
            let ent_path = if C == 1 {
                format!(
                    "{ent_path}/{}",
                    row_names
                        .as_ref()
                        .map(|n| n[r].clone())
                        .unwrap_or(format!("{}", r + 1))
                )
            } else {
                format!(
                    "{ent_path}/{}{}",
                    row_names
                        .as_ref()
                        .map(|n| n[r].clone())
                        .unwrap_or(format!("{}", r + 1)),
                    col_names
                        .as_ref()
                        .map(|n| n[c].clone())
                        .unwrap_or(format!("{}", c + 1))
                )
            };

            rec.log(ent_path, &rerun::Scalars::single(matrix[(r, c)].as_()))?;
        }
    }

    Ok(())
}

fn _log_matrix_tensor<const R: usize, const C: usize>(
    rec: &mut RecordingStream,
    ent_path: String,
    matrix: &SMatrix<f64, R, C>,
) -> Result<()> {
    let tensor = TensorData::new(
        vec![R as u64, C as u64],
        rerun::TensorBuffer::F64(ScalarBuffer::from(matrix.as_slice().to_vec())),
    );

    let tensor = rerun::Tensor::new(tensor);
    rec.log(ent_path, &tensor)?;

    Ok(())
}

fn log_vector3_timeseries<T: Float + AsPrimitive<f64>>(
    rec: &mut RecordingStream,
    ent_path: String,
    matrix: &Vector3<T>,
) -> Result<()> {
    let row_names = ["x".to_string(), "y".to_string(), "z".to_string()];
    log_matrix_timeseries(rec, ent_path, matrix, Some(&row_names), None)
}

fn log_quat_timeseries<T: RealField + Float + AsPrimitive<f64>>(
    rec: &mut RecordingStream,
    quat: &UnitQuaternion<T>,
    quat_ent_path: String,
    eul_ent_path: String,
) -> Result<()> {
    let quat_row_names = [
        "w".to_string(),
        "x".to_string(),
        "y".to_string(),
        "z".to_string(),
    ];
    let quat_vec: &Vector4<T> = quat.quaternion().as_vector();
    log_matrix_timeseries(rec, quat_ent_path, quat_vec, Some(&quat_row_names), None)?;

    let eul_row_names = ["yaw".to_string(), "pitch".to_string(), "roll".to_string()];
    let (roll, pitch, yaw) = quat.euler_angles();
    log_matrix_timeseries(
        rec,
        eul_ent_path,
        &Vector3::new(yaw.to_degrees(), pitch.to_degrees(), roll.to_degrees()),
        Some(&eul_row_names),
        None,
    )?;

    Ok(())
}

fn log_matrix3_timeseries<T: Float + AsPrimitive<f64>>(
    rec: &mut RecordingStream,
    ent_path: String,
    matrix: &Matrix3<T>,
) -> Result<()> {
    let row_names = ["x".to_string(), "y".to_string(), "z".to_string()];

    log_matrix_timeseries(rec, ent_path, matrix, Some(&row_names), Some(&row_names))
}
