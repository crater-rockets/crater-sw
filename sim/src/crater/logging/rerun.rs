use crate::{
    core::time::Timestamp,
    crater::sim::{
        gnc::ServoPosition,
        rocket_data::{AeroAngles, RocketActions, RocketState},
    },
    telemetry::{TelemetryDispatcher, TelemetryReceiver, TelemetryService, Timestamped},
    utils::{capacity::Capacity, ringchannel::Select},
};
use anyhow::Result;
use map_3d::ned2geodetic;
use nalgebra::Vector3;
use rerun::{components::RotationQuat, Quaternion, RecordingStream};

pub struct RerunLogger {
    receivers: Receivers,
}

struct Receivers {
    rocket_state: TelemetryReceiver<RocketState>,
    rocket_actions: TelemetryReceiver<RocketActions>,
    aero_angles: TelemetryReceiver<AeroAngles>,
    control_servo_cmd: TelemetryReceiver<ServoPosition>,
    actuator_servo_pos: TelemetryReceiver<ServoPosition>,
}
pub struct RerunLoggerConnection {
    rx: Receivers,
    rec: RecordingStream,

    memory: Memory,
    origin: [f64; 3],
}

#[derive(Debug, Default, Clone)]
struct Memory {
    trajectory_ned_3d: Vec<[f32; 3]>,
    trajectory_geodetic: Vec<[f64; 2]>,
    ts_last_element: f64,
}

impl RerunLogger {
    pub fn new(ts: &TelemetryService) -> Result<Self> {
        let receivers = Receivers {
            rocket_state: ts.subscribe("/rocket/state", Capacity::Unbounded)?,
            rocket_actions: ts.subscribe("/rocket/actions", Capacity::Unbounded)?,
            aero_angles: ts.subscribe("/rocket/aero_angles", Capacity::Unbounded)?,
            control_servo_cmd: ts.subscribe("/gnc/control/servo_command", Capacity::Unbounded)?,
            actuator_servo_pos: ts.subscribe("/actuators/servo_position", Capacity::Unbounded)?,
        };

        Ok(RerunLogger { receivers })
    }

    pub fn connect(self) -> Result<RerunLoggerConnection> {
        let rec = rerun::RecordingStreamBuilder::new("crater").connect_tcp()?;

        // Initialize frame of reference
        rec.log_static("/", &rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN)?;

        rec.set_time_seconds("sim_time", 0.0);

        rec.log(
            "rocket",
            &rerun::Asset3D::from_file("assets/sidewinder.obj")?,
        )?;

        let conn = RerunLoggerConnection {
            rx: self.receivers,
            rec,
            memory: Memory::default(),
            origin: [
                f64::to_radians(41.8080239),
                f64::to_radians(14.0548082),
                1411.211,
            ],
        };

        Ok(conn)
    }
}

impl RerunLoggerConnection {
    pub fn log_blocking(&mut self) -> Result<()> {
        let mut select = Select::default();

        let i_rocket_state = select.add(&self.rx.rocket_state);
        let i_rcv_rocket_actions = select.add(&self.rx.rocket_actions);
        let i_rcv_aero_angles = select.add(&self.rx.aero_angles);
        let i_control_servo_cmd = select.add(&self.rx.control_servo_cmd);
        let i_actuator_servo_pos = select.add(&self.rx.actuator_servo_pos);

        while select.num_active_subs() > 0 {
            let i = select.ready();

            match i {
                i if i == i_rocket_state => {
                    if let Ok(Timestamped(ts, state)) = self.rx.rocket_state.recv() {
                        Self::log_rocket_state(&mut self.rec, &mut self.memory, ts, state)?;
                    } else {
                        select.remove(i);
                    }
                }
                i if i == i_rcv_rocket_actions => {
                    if let Ok(Timestamped(ts, state)) = self.rx.rocket_actions.recv() {
                        Self::log_rocket_actions(&mut self.rec, &mut self.memory, ts, state)?;
                    } else {
                        select.remove(i);
                    }
                }
                i if i == i_rcv_aero_angles => {
                    if let Ok(Timestamped(ts, state)) = self.rx.aero_angles.recv() {
                        Self::log_aero_angles(&mut self.rec, &mut self.memory, ts, state)?;
                    } else {
                        select.remove(i);
                    }
                }
                i if i == i_actuator_servo_pos => {
                    if let Ok(Timestamped(ts, servo_pos)) = self.rx.actuator_servo_pos.recv() {
                        Self::log_servo_pos(
                            "/timeseries/servo_position",
                            &mut self.rec,
                            &mut self.memory,
                            ts,
                            servo_pos,
                        )?;
                    } else {
                        select.remove(i);
                    }
                }
                i if i == i_control_servo_cmd => {
                    if let Ok(Timestamped(ts, servo_pos)) = self.rx.control_servo_cmd.recv() {
                        Self::log_servo_pos(
                            "/timeseries/servo_command",
                            &mut self.rec,
                            &mut self.memory,
                            ts,
                            servo_pos,
                        )?;
                    } else {
                        select.remove(i);
                    }
                }
                _ => {
                    unreachable!()
                }
            }
        }

        Ok(())
    }

    fn log_rocket_state(
        rec: &mut RecordingStream,
        memory: &mut Memory,
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

        // Position
        rec.log("timeseries/position/x", &rerun::Scalar::new(pos[0]))?;
        rec.log("timeseries/position/y", &rerun::Scalar::new(pos[1]))?;
        rec.log("timeseries/position/z", &rerun::Scalar::new(pos[2]))?;

        // Velocity
        let vel_b = state.vel_b(&state.quat_nb());
        let vnorm = vel_b.norm();

        rec.log("timeseries/velocity/body/x", &rerun::Scalar::new(vel_b[0]))?;
        rec.log("timeseries/velocity/body/y", &rerun::Scalar::new(vel_b[1]))?;
        rec.log("timeseries/velocity/body/z", &rerun::Scalar::new(vel_b[2]))?;

        rec.log("timeseries/velocity/norm", &rerun::Scalar::new(vnorm))?;

        // Angular speed
        let angvel = state.angvel_b();
        rec.log(
            "timeseries/angular_speed/x",
            &rerun::Scalar::new(angvel[0].to_degrees()),
        )?;
        rec.log(
            "timeseries/angular_speed/y",
            &rerun::Scalar::new(angvel[1].to_degrees()),
        )?;
        rec.log(
            "timeseries/angular_speed/z",
            &rerun::Scalar::new(angvel[2].to_degrees()),
        )?;

        // Orientation
        let quat = state.quat_nb();
        rec.log("timeseries/orientation/quat/x", &rerun::Scalar::new(quat.i))?;
        rec.log("timeseries/orientation/quat/y", &rerun::Scalar::new(quat.j))?;
        rec.log("timeseries/orientation/quat/z", &rerun::Scalar::new(quat.k))?;
        rec.log("timeseries/orientation/quat/w", &rerun::Scalar::new(quat.w))?;

        let euler = quat.euler_angles();
        rec.log(
            "timeseries/orientation/euler/yaw",
            &rerun::Scalar::new(euler.2.to_degrees()),
        )?;
        rec.log(
            "timeseries/orientation/euler/pitch",
            &rerun::Scalar::new(euler.1.to_degrees()),
        )?;
        rec.log(
            "timeseries/orientation/euler/roll",
            &rerun::Scalar::new(euler.0.to_degrees()),
        )?;

        // Log trajectory less frequently, as every log contains the full position history, making logs huge
        if memory.ts_last_element == 0.0 || ts_seconds - memory.ts_last_element >= 0.1 {
            memory.ts_last_element = ts_seconds;

            // Keep history of 3D position to display a 3D trajectory in Rerun
            memory
                .trajectory_ned_3d
                .push([pos[0] as f32, pos[1] as f32, pos[2] as f32]);

            // Keep history of latitude and longitude to display trajectory over a map
            memory
                .trajectory_geodetic
                .push([lat.to_degrees(), lon.to_degrees()]);

            // Trajectory lines
            rec.log(
                "trajectory/ned_3d",
                &rerun::LineStrips3D::new([memory.trajectory_ned_3d.as_slice()]),
            )?;

            rec.log(
                "trajectory/geodetic",
                &rerun::GeoLineStrings::from_lat_lon([memory.trajectory_geodetic.as_slice()]),
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
            &rerun::Arrows3D::from_vectors([vec3_to_slice(&(state.vel_b(&state.quat_nb()) / 10.0))])
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

    fn log_aero_angles(
        rec: &mut RecordingStream,
        _: &mut Memory,
        ts: Timestamp,
        angles: AeroAngles,
    ) -> Result<()> {
        rec.set_time_seconds("sim_time", ts.monotonic.elapsed_seconds_f64());

        rec.log(
            "timeseries/aero/alpha",
            &rerun::Scalar::new(angles.alpha.to_degrees()),
        )?;

        rec.log(
            "timeseries/aero/beta",
            &rerun::Scalar::new(angles.beta.to_degrees()),
        )?;

        Ok(())
    }

    fn log_rocket_actions(
        rec: &mut RecordingStream,
        _: &mut Memory,
        ts: Timestamp,
        actions: RocketActions,
    ) -> Result<()> {
        rec.set_time_seconds("sim_time", ts.monotonic.elapsed_seconds_f64());

        rec.log(
            "timeseries/actions/body/thrust/x",
            &rerun::Scalar::new(actions.thrust_b[0]),
        )?;
        rec.log(
            "timeseries/actions/body/thrust/y",
            &rerun::Scalar::new(actions.thrust_b[1]),
        )?;
        rec.log(
            "timeseries/actions/body/thrust/z",
            &rerun::Scalar::new(actions.thrust_b[2]),
        )?;

        rec.log(
            "timeseries/actions/body/aero_force/x",
            &rerun::Scalar::new(actions.aero_force_b[0]),
        )?;
        rec.log(
            "timeseries/actions/body/aero_force/y",
            &rerun::Scalar::new(actions.aero_force_b[1]),
        )?;
        rec.log(
            "timeseries/actions/body/aero_force/z",
            &rerun::Scalar::new(actions.aero_force_b[2]),
        )?;

        rec.log(
            "timeseries/actions/body/aero_torque/x",
            &rerun::Scalar::new(actions.aero_torque_b[0]),
        )?;
        rec.log(
            "timeseries/actions/body/aero_torque/y",
            &rerun::Scalar::new(actions.aero_torque_b[1]),
        )?;
        rec.log(
            "timeseries/actions/body/aero_torque/z",
            &rerun::Scalar::new(actions.aero_torque_b[2]),
        )?;

        rec.log(
            "timeseries/actions/body/acc/x",
            &rerun::Scalar::new(actions.acc_b[0]),
        )?;
        rec.log(
            "timeseries/actions/body/acc/y",
            &rerun::Scalar::new(actions.acc_b[1]),
        )?;
        rec.log(
            "timeseries/actions/body/acc/z",
            &rerun::Scalar::new(actions.acc_b[2]),
        )?;

        rec.log(
            "timeseries/actions/ned/acc/x",
            &rerun::Scalar::new(actions.acc_n[0]),
        )?;
        rec.log(
            "timeseries/actions/ned/acc/y",
            &rerun::Scalar::new(actions.acc_n[1]),
        )?;
        rec.log(
            "timeseries/actions/ned/acc/z",
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

    fn log_servo_pos(
        ent_path: &str,
        rec: &mut RecordingStream,
        _: &mut Memory,
        ts: Timestamp,
        servo_pos: ServoPosition,
    ) -> Result<()> {
        rec.set_time_seconds("sim_time", ts.monotonic.elapsed_seconds_f64());

        rec.log(
            format!("{}/raw/1", ent_path),
            &rerun::Scalar::new(servo_pos.0[0].to_degrees()),
        )?;
        rec.log(
            format!("{}/raw/2", ent_path),
            &rerun::Scalar::new(servo_pos.0[1].to_degrees()),
        )?;
        rec.log(
            format!("{}/raw/3", ent_path),
            &rerun::Scalar::new(servo_pos.0[2].to_degrees()),
        )?;
        rec.log(
            format!("{}/raw/4", ent_path),
            &rerun::Scalar::new(servo_pos.0[3].to_degrees()),
        )?;

        let mixed = servo_pos.mix();

        rec.log(
            format!("{}/mixed/yaw", ent_path),
            &rerun::Scalar::new(mixed.yaw().to_degrees()),
        )?;
        rec.log(
            format!("{}/mixed/pitch", ent_path),
            &rerun::Scalar::new(mixed.pitch().to_degrees()),
        )?;
        rec.log(
            format!("{}/mixed/roll", ent_path),
            &rerun::Scalar::new(mixed.roll().to_degrees()),
        )?;
        rec.log(
            format!("{}/mixed/squeeze", ent_path),
            &rerun::Scalar::new(mixed.squeeze().to_degrees()),
        )?;

        Ok(())
    }
}

fn vec3_to_slice(vec: &Vector3<f64>) -> [f32; 3] {
    [vec[0] as f32, vec[1] as f32, vec[2] as f32]
}
