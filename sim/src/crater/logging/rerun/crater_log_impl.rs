use map_3d::ned2geodetic;
use nalgebra::Vector3;
use rerun::{components::RotationQuat, Quaternion, RecordingStream};

use crate::{
    core::time::Timestamp,
    crater::sim::{
        gnc::ServoPosition,
        rocket_data::{AeroAngles, RocketActions, RocketMassProperties, RocketState},
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
        rec.set_time_seconds("sim_time", ts_seconds);

        let pos = state.pos_n();
        rec.log(format!("{ent_path}/pos_n/x"), &rerun::Scalar::new(pos[0]))?;
        rec.log(format!("{ent_path}/pos_n/y"), &rerun::Scalar::new(pos[1]))?;
        rec.log(format!("{ent_path}/pos_n/z"), &rerun::Scalar::new(pos[2]))?;

        let vel_n = state.vel_n();
        rec.log(format!("{ent_path}/vel_n/x"), &rerun::Scalar::new(vel_n[0]))?;
        rec.log(format!("{ent_path}/vel_n/y"), &rerun::Scalar::new(vel_n[1]))?;
        rec.log(format!("{ent_path}/vel_n/z"), &rerun::Scalar::new(vel_n[2]))?;

        let angvel = state.angvel_b();
        rec.log(
            format!("{ent_path}/ang_vel_b/x"),
            &rerun::Scalar::new(angvel[0].to_degrees()),
        )?;
        rec.log(
            format!("{ent_path}/ang_vel_b/y"),
            &rerun::Scalar::new(angvel[1].to_degrees()),
        )?;
        rec.log(
            format!("{ent_path}/ang_vel_b/z"),
            &rerun::Scalar::new(angvel[2].to_degrees()),
        )?;

        let quat = state.quat_nb();
        rec.log(
            format!("{ent_path}/orient/quat/x"),
            &rerun::Scalar::new(quat.i),
        )?;
        rec.log(
            format!("{ent_path}/orient/quat/y"),
            &rerun::Scalar::new(quat.j),
        )?;
        rec.log(
            format!("{ent_path}/orient/quat/z"),
            &rerun::Scalar::new(quat.k),
        )?;
        rec.log(
            format!("{ent_path}/orient/quat/w"),
            &rerun::Scalar::new(quat.w),
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
        let pos = state.pos_n();
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
        rec.set_time_seconds("sim_time", ts_seconds);

        rec.log(
            "/frame/body_centered",
            &rerun::Transform3D::from_translation(pos_f32),
        )?;

        // Velocity body frame
        let vel_b = state.vel_b(&state.quat_nb());
        let vnorm = vel_b.norm();

        rec.log(format!("{ent_path}/vel_b/x"), &rerun::Scalar::new(vel_b[0]))?;
        rec.log(format!("{ent_path}/vel_b/y"), &rerun::Scalar::new(vel_b[1]))?;
        rec.log(format!("{ent_path}/vel_b/z"), &rerun::Scalar::new(vel_b[2]))?;

        rec.log(format!("{ent_path}/vel_norm"), &rerun::Scalar::new(vnorm))?;

        // Orientation Euler
        let quat = state.quat_nb();
        let euler = quat.euler_angles();
        rec.log(
            format!("{ent_path}/orient/euler/yaw"),
            &rerun::Scalar::new(euler.2.to_degrees()),
        )?;
        rec.log(
            format!("{ent_path}/orient/euler/pitch"),
            &rerun::Scalar::new(euler.1.to_degrees()),
        )?;
        rec.log(
            format!("{ent_path}/orient/euler/roll"),
            &rerun::Scalar::new(euler.0.to_degrees()),
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
                &(state.vel_b(&state.quat_nb()) / 10.0),
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
pub struct AeroAnglesLog;

impl RerunWrite for AeroAnglesLog {
    type Telem = AeroAngles;

    fn write(
        &mut self,
        rec: &mut RecordingStream,
        ent_path: &str,
        ts: Timestamp,
        angles: AeroAngles,
    ) -> Result<()> {
        rec.set_time_seconds("sim_time", ts.monotonic.elapsed_seconds_f64());

        rec.log(
            format!("{ent_path}/alpha"),
            &rerun::Scalar::new(angles.alpha.to_degrees()),
        )?;

        rec.log(
            format!("{ent_path}/beta"),
            &rerun::Scalar::new(angles.beta.to_degrees()),
        )?;

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
        rec.set_time_seconds("sim_time", ts.monotonic.elapsed_seconds_f64());

        rec.log(
            format!("{ent_path}/thrust_b/x"),
            &rerun::Scalar::new(actions.thrust_b[0]),
        )?;
        rec.log(
            format!("{ent_path}/thrust_b/y"),
            &rerun::Scalar::new(actions.thrust_b[1]),
        )?;
        rec.log(
            format!("{ent_path}/thrust_b/z"),
            &rerun::Scalar::new(actions.thrust_b[2]),
        )?;

        rec.log(
            format!("{ent_path}/aero_force_b/x"),
            &rerun::Scalar::new(actions.aero_force_b[0]),
        )?;
        rec.log(
            format!("{ent_path}/aero_force_b/y"),
            &rerun::Scalar::new(actions.aero_force_b[1]),
        )?;
        rec.log(
            format!("{ent_path}/aero_force_b/z"),
            &rerun::Scalar::new(actions.aero_force_b[2]),
        )?;

        rec.log(
            format!("{ent_path}/aero_torque_b/x"),
            &rerun::Scalar::new(actions.aero_torque_b[0]),
        )?;
        rec.log(
            format!("{ent_path}/aero_torque_b/y"),
            &rerun::Scalar::new(actions.aero_torque_b[1]),
        )?;
        rec.log(
            format!("{ent_path}/aero_torque_b/z"),
            &rerun::Scalar::new(actions.aero_torque_b[2]),
        )?;

        rec.log(
            format!("{ent_path}/acc_b/x"),
            &rerun::Scalar::new(actions.acc_b[0]),
        )?;
        rec.log(
            format!("{ent_path}/acc_b/y"),
            &rerun::Scalar::new(actions.acc_b[1]),
        )?;
        rec.log(
            format!("{ent_path}/acc_b/z"),
            &rerun::Scalar::new(actions.acc_b[2]),
        )?;

        rec.log(
            format!("{ent_path}/acc_n/x"),
            &rerun::Scalar::new(actions.acc_n[0]),
        )?;
        rec.log(
            format!("{ent_path}/acc_n/y"),
            &rerun::Scalar::new(actions.acc_n[1]),
        )?;
        rec.log(
            format!("{ent_path}/acc_n/z"),
            &rerun::Scalar::new(actions.acc_n[2]),
        )?;

        let thrust_scaled = actions.thrust_b / 20.0;
        let aero_force_scaled = actions.aero_force_b / 1.0;

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
        rec.set_time_seconds("sim_time", ts.monotonic.elapsed_seconds_f64());

        rec.log(
            format!("{ent_path}/raw/1"),
            &rerun::Scalar::new(servo_pos.0[0].to_degrees()),
        )?;
        rec.log(
            format!("{ent_path}/raw/2"),
            &rerun::Scalar::new(servo_pos.0[1].to_degrees()),
        )?;
        rec.log(
            format!("{ent_path}/raw/3"),
            &rerun::Scalar::new(servo_pos.0[2].to_degrees()),
        )?;
        rec.log(
            format!("{ent_path}/raw/4"),
            &rerun::Scalar::new(servo_pos.0[3].to_degrees()),
        )?;

        let mixed = servo_pos.mix();

        rec.log(
            format!("{ent_path}/mixed/yaw"),
            &rerun::Scalar::new(mixed.yaw().to_degrees()),
        )?;
        rec.log(
            format!("{ent_path}/mixed/pitch"),
            &rerun::Scalar::new(mixed.pitch().to_degrees()),
        )?;
        rec.log(
            format!("{ent_path}/mixed/roll"),
            &rerun::Scalar::new(mixed.roll().to_degrees()),
        )?;
        rec.log(
            format!("{ent_path}/mixed/squeeze"),
            &rerun::Scalar::new(mixed.squeeze().to_degrees()),
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
        rec.set_time_seconds("sim_time", ts.monotonic.elapsed_seconds_f64());

        rec.log(
            "timeseries/masses/xcg_tot/x",
            &rerun::Scalar::new(mass.xcg_total[0]),
        )?;

        rec.log(
            "timeseries/masses/xcg_tot/y",
            &rerun::Scalar::new(mass.xcg_total[1]),
        )?;

        rec.log(
            "timeseries/masses/xcg_tot/z",
            &rerun::Scalar::new(mass.xcg_total[2]),
        )?;

        rec.log(
            "timeseries/masses/mass_tot",
            &rerun::Scalar::new(mass.mass_tot),
        )?;

        rec.log(
            "timeseries/masses/mass_dot",
            &rerun::Scalar::new(mass.mass_dot),
        )?;

        rec.log(
            "timeseries/masses/inertia/xx",
            &rerun::Scalar::new(mass.inertia[0]),
        )?;

        rec.log(
            "timeseries/masses/inertia/xy",
            &rerun::Scalar::new(mass.inertia[1]),
        )?;

        rec.log(
            "timeseries/masses/inertia/xz",
            &rerun::Scalar::new(mass.inertia[2]),
        )?;

        rec.log(
            "timeseries/masses/inertia/yx",
            &rerun::Scalar::new(mass.inertia[3]),
        )?;

        rec.log(
            "timeseries/masses/inertia/yy",
            &rerun::Scalar::new(mass.inertia[4]),
        )?;

        rec.log(
            "timeseries/masses/inertia/yz",
            &rerun::Scalar::new(mass.inertia[5]),
        )?;

        rec.log(
            "timeseries/masses/inertia/zx",
            &rerun::Scalar::new(mass.inertia[6]),
        )?;

        rec.log(
            "timeseries/masses/inertia/zy",
            &rerun::Scalar::new(mass.inertia[7]),
        )?;

        rec.log(
            "timeseries/masses/inertia/zz",
            &rerun::Scalar::new(mass.inertia[8]),
        )?;

        Ok(())
    }
}

fn vec3_to_slice(vec: &Vector3<f64>) -> [f32; 3] {
    [vec[0] as f32, vec[1] as f32, vec[2] as f32]
}
