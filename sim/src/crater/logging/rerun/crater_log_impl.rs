use crater_gnc::components::ada::AdaResult;
use map_3d::ned2geodetic;
use nalgebra::{Matrix3, SMatrix, Vector3};
use rerun::{
    Quaternion, RecordingStream, TensorData, TextLogLevel, components::RotationQuat,
    external::arrow::buffer::ScalarBuffer,
};
use uom::si::{length::meter, velocity::meter_per_second};

use crate::{
    core::time::Timestamp,
    crater::sim::{
        aero::aerodynamics::AeroState,
        engine::engine::RocketEngineMassProperties,
        events::{GncEventItem, SimEvent},
        gnc::ServoPosition,
        rocket::{
            mass::RocketMassProperties,
            rocket_data::{RocketAccelerations, RocketActions, RocketState},
        },
        sensors::{IMUSample, MagnetometerSample},
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
        ent_path: &str,
        ts: Timestamp,
        state: RocketState,
    ) -> Result<()> {
        let ts_seconds = ts.monotonic.elapsed_seconds_f64();
        rec.set_duration_secs("sim_time", ts_seconds);

        let pos = state.pos_n_m();
        rec.log(
            format!("{ent_path}/pos_n/x"),
            &rerun::Scalars::single(pos[0]),
        )?;
        rec.log(
            format!("{ent_path}/pos_n/y"),
            &rerun::Scalars::single(pos[1]),
        )?;
        rec.log(
            format!("{ent_path}/pos_n/z"),
            &rerun::Scalars::single(pos[2]),
        )?;

        let vel_n = state.vel_n_m_s();
        rec.log(
            format!("{ent_path}/vel_n/x"),
            &rerun::Scalars::single(vel_n[0]),
        )?;
        rec.log(
            format!("{ent_path}/vel_n/y"),
            &rerun::Scalars::single(vel_n[1]),
        )?;
        rec.log(
            format!("{ent_path}/vel_n/z"),
            &rerun::Scalars::single(vel_n[2]),
        )?;

        let angvel = state.angvel_b_rad_s();
        rec.log(
            format!("{ent_path}/ang_vel_b/x"),
            &rerun::Scalars::single(angvel[0].to_degrees()),
        )?;
        rec.log(
            format!("{ent_path}/ang_vel_b/y"),
            &rerun::Scalars::single(angvel[1].to_degrees()),
        )?;
        rec.log(
            format!("{ent_path}/ang_vel_b/z"),
            &rerun::Scalars::single(angvel[2].to_degrees()),
        )?;

        let quat = state.quat_nb();
        rec.log(
            format!("{ent_path}/orient/quat/x"),
            &rerun::Scalars::single(quat.i),
        )?;
        rec.log(
            format!("{ent_path}/orient/quat/y"),
            &rerun::Scalars::single(quat.j),
        )?;
        rec.log(
            format!("{ent_path}/orient/quat/z"),
            &rerun::Scalars::single(quat.k),
        )?;
        rec.log(
            format!("{ent_path}/orient/quat/w"),
            &rerun::Scalars::single(quat.w),
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
        let pos_f32 = vec3_to_slice(&pos.clone_owned());
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
        rec.set_duration_secs("sim_time", ts_seconds);

        rec.log(
            "/frame/body_centered",
            &rerun::Transform3D::from_translation(pos_f32),
        )?;

        // Velocity body frame
        let vel_b = state.vel_b_m_s(&state.quat_nb());
        let vnorm = vel_b.norm();

        rec.log(
            format!("{ent_path}/vel_b/x"),
            &rerun::Scalars::single(vel_b[0]),
        )?;
        rec.log(
            format!("{ent_path}/vel_b/y"),
            &rerun::Scalars::single(vel_b[1]),
        )?;
        rec.log(
            format!("{ent_path}/vel_b/z"),
            &rerun::Scalars::single(vel_b[2]),
        )?;

        rec.log(
            format!("{ent_path}/vel_norm"),
            &rerun::Scalars::single(vnorm),
        )?;

        // Orientation Euler
        let quat = state.quat_nb();
        let euler = quat.euler_angles();
        rec.log(
            format!("{ent_path}/orient/euler/yaw"),
            &rerun::Scalars::single(euler.2.to_degrees()),
        )?;
        rec.log(
            format!("{ent_path}/orient/euler/pitch"),
            &rerun::Scalars::single(euler.1.to_degrees()),
        )?;
        rec.log(
            format!("{ent_path}/orient/euler/roll"),
            &rerun::Scalars::single(euler.0.to_degrees()),
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
        rec.log(
            "objects/vectors/velocity",
            &rerun::Arrows3D::from_vectors([vec3_to_slice(
                &(state.vel_b_m_s(&state.quat_nb()) / 10.0),
            )])
            .with_colors([rerun::Color::from_rgb(0, 255, 0)])
            .with_origins([[0.0, 0.0, 0.0]]),
        )?;

        // Transform
        let body_transform = rerun::Transform3D::from_translation_rotation(
            pos_f32,
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
        ent_path: &str,
        ts: Timestamp,
        state: AeroState,
    ) -> Result<()> {
        rec.set_duration_secs("sim_time", ts.monotonic.elapsed_seconds_f64());

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

        log_vector3_timeseries(rec, format!("{ent_path}/v_air_b_m_s"), state.v_air_b_m_s)?;
        log_vector3_timeseries(rec, format!("{ent_path}/w_b_rad_s"), state.w_b_rad_s)?;

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
        ent_path: &str,
        ts: Timestamp,
        actions: RocketActions,
    ) -> Result<()> {
        rec.set_duration_secs("sim_time", ts.monotonic.elapsed_seconds_f64());

        log_vector3_timeseries(rec, format!("{ent_path}/thrust_b_n"), actions.thrust_b_n)?;
        log_vector3_timeseries(
            rec,
            format!("{ent_path}/aero_force_b_n"),
            actions.aero_actions.forces_b_n,
        )?;
        log_vector3_timeseries(
            rec,
            format!("{ent_path}/aero_moments_b_nm"),
            actions.aero_actions.moments_b_nm,
        )?;

        let thrust_scaled = actions.thrust_b_n / 20.0;
        let aero_force_scaled = actions.aero_actions.forces_b_n / 1.0;

        rec.log(
            "objects/vectors/thurst",
            &rerun::Arrows3D::from_vectors([vec3_to_slice(&thrust_scaled)])
                .with_colors([rerun::Color::from_rgb(255, 0, 0)])
                .with_origins([[2.0, 0.0, 0.0]]),
        )?;

        rec.log(
            "objects/vectors/aero_forces",
            &rerun::Arrows3D::from_vectors([vec3_to_slice(&aero_force_scaled)])
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
        ent_path: &str,
        ts: Timestamp,
        accel: RocketAccelerations,
    ) -> Result<()> {
        rec.set_duration_secs("sim_time", ts.monotonic.elapsed_seconds_f64());

        log_vector3_timeseries(rec, format!("{ent_path}/acc_b"), accel.acc_b_m_s2)?;
        log_vector3_timeseries(rec, format!("{ent_path}/acc_n"), accel.acc_n_m_s2)?;

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
        ent_path: &str,
        ts: Timestamp,
        servo_pos: ServoPosition,
    ) -> Result<()> {
        rec.set_duration_secs("sim_time", ts.monotonic.elapsed_seconds_f64());

        log_matrix_timeseries(
            rec,
            format!("{ent_path}/raw"),
            servo_pos.pos_rad.map(|x| x.to_degrees()),
            None,
            None,
        )?;

        let mixed = servo_pos.mix();
        log_matrix_timeseries(
            rec,
            format!("{ent_path}/mixed"),
            mixed.pos_rad.map(|x| x.to_degrees()),
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
        ent_path: &str,
        ts: Timestamp,
        mass: RocketMassProperties,
    ) -> Result<()> {
        rec.set_duration_secs("sim_time", ts.monotonic.elapsed_seconds_f64());

        log_vector3_timeseries(rec, format!("{ent_path}/xcg_tot_m"), mass.xcg_total_m)?;

        rec.log(
            format!("{ent_path}/mass_tot_kg"),
            &rerun::Scalars::single(mass.mass_kg),
        )?;

        rec.log(
            format!("{ent_path}/mass_dot_kg_s"),
            &rerun::Scalars::single(mass.mass_dot_kg_s),
        )?;

        log_matrix3_timeseries(rec, format!("{ent_path}/inertia_kgm2"), mass.inertia_kgm2)?;
        log_matrix3_timeseries(
            rec,
            format!("{ent_path}/inertia_dot_kgm2_s"),
            mass.inertia_dot_kgm2_s,
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
        ent_path: &str,
        ts: Timestamp,
        mass: RocketEngineMassProperties,
    ) -> Result<()> {
        rec.set_duration_secs("sim_time", ts.monotonic.elapsed_seconds_f64());

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
            mass.inertia_eng_frame_kgm2,
        )?;
        log_matrix3_timeseries(
            rec,
            format!("{ent_path}/inertia_dot_eng_frame_kgm2"),
            mass.inertia_dot_eng_frame_kgm2,
        )?;

        Ok(())
    }
}

#[derive(Default)]
pub struct IMUSampleLog;

impl RerunWrite for IMUSampleLog {
    type Telem = IMUSample;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        ent_path: &str,
        ts: Timestamp,
        imu: IMUSample,
    ) -> Result<()> {
        rec.set_duration_secs("sim_time", ts.monotonic.elapsed_seconds_f64());

        log_vector3_timeseries(rec, format!("{ent_path}/acc_m_s2"), imu.acc)?;

        let gyro_deg = imu.gyro.map(|x| x.to_degrees());
        log_vector3_timeseries(rec, format!("{ent_path}/gyro_deg_s"), gyro_deg)?;

        Ok(())
    }
}

#[derive(Default)]
pub struct MagnetometerSampleLog;

impl RerunWrite for MagnetometerSampleLog {
    type Telem = MagnetometerSample;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        ent_path: &str,
        ts: Timestamp,
        mag: MagnetometerSample,
    ) -> Result<()> {
        rec.set_duration_secs("sim_time", ts.monotonic.elapsed_seconds_f64());

        log_vector3_timeseries(rec, ent_path.to_string(), mag.magfield_b)?;

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
        ent_path: &str,
        ts: Timestamp,
        event: GncEventItem,
    ) -> Result<()> {
        rec.set_duration_secs("sim_time", ts.monotonic.elapsed_seconds_f64());

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
        ent_path: &str,
        ts: Timestamp,
        event: SimEvent,
    ) -> Result<()> {
        rec.set_duration_secs("sim_time", ts.monotonic.elapsed_seconds_f64());

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
        ent_path: &str,
        ts: Timestamp,
        ada: AdaResult,
    ) -> Result<()> {
        rec.set_duration_secs("sim_time", ts.monotonic.elapsed_seconds_f64());

        rec.log(
            format!("{}/altitude_m", ent_path),
            &rerun::Scalars::single(ada.altitude.get::<meter>() as f64),
        )?;

        rec.log(
            format!("{}/vertical_speed_m_s", ent_path),
            &rerun::Scalars::single(ada.vertical_speed.get::<meter_per_second>() as f64),
        )?;

        Ok(())
    }
}

fn vec3_to_slice(vec: &Vector3<f64>) -> [f32; 3] {
    [vec[0] as f32, vec[1] as f32, vec[2] as f32]
}

fn log_matrix_timeseries<const R: usize, const C: usize>(
    rec: &mut RecordingStream,
    ent_path: String,
    matrix: SMatrix<f64, R, C>,
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

            rec.log(ent_path, &rerun::Scalars::single(matrix[(r, c)]))?;
        }
    }

    Ok(())
}

fn _log_matrix_tensor<const R: usize, const C: usize>(
    rec: &mut RecordingStream,
    ent_path: String,
    matrix: SMatrix<f64, R, C>,
) -> Result<()> {
    let tensor = TensorData::new(
        vec![R as u64, C as u64],
        rerun::TensorBuffer::F64(ScalarBuffer::from(matrix.as_slice().to_vec())),
    );

    let tensor = rerun::Tensor::new(tensor);
    rec.log(ent_path, &tensor)?;

    Ok(())
}

fn log_vector3_timeseries(
    rec: &mut RecordingStream,
    ent_path: String,
    matrix: Vector3<f64>,
) -> Result<()> {
    let row_names = ["x".to_string(), "y".to_string(), "z".to_string()];
    log_matrix_timeseries(rec, ent_path, matrix, Some(&row_names), None)
}

fn log_matrix3_timeseries(
    rec: &mut RecordingStream,
    ent_path: String,
    matrix: Matrix3<f64>,
) -> Result<()> {
    let row_names = ["x".to_string(), "y".to_string(), "z".to_string()];

    log_matrix_timeseries(rec, ent_path, matrix, Some(&row_names), Some(&row_names))
}
