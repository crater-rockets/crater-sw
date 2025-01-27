use crate::{
    core::time::Timestamp,
    crater::sim::rocket_data::{AeroAngles, RocketActions, RocketState},
    telemetry::{TelemetryDispatcher, TelemetryReceiver, TelemetryService, Timestamped},
    utils::{capacity::Capacity, ringchannel::Select},
};
use anyhow::Result;
use map_3d::ned2geodetic;
use rerun::{components::RotationQuat, Quaternion, RecordingStream};

pub struct RerunLogger {
    rocket_state: TelemetryReceiver<RocketState>,
    rocket_actions: TelemetryReceiver<RocketActions>,
    aero_angles: TelemetryReceiver<AeroAngles>,
}

pub struct RerunLoggerConnection {
    rcv_rocket_state: TelemetryReceiver<RocketState>,
    rcv_rocket_actions: TelemetryReceiver<RocketActions>,
    rcv_aero_angles: TelemetryReceiver<AeroAngles>,

    rec: RecordingStream,

    memory: Memory,
    origin: [f64; 3],
}

#[derive(Debug, Default, Clone)]
struct Memory {
    trajectory_ned_3d: Vec<[f32; 3]>,
    trajectory_geodetic: Vec<[f64; 2]>,
}

impl RerunLogger {
    pub fn new(ts: &TelemetryService) -> Result<Self> {
        let rocket_state = ts.subscribe::<RocketState>("/rocket/state", Capacity::Unbounded)?;
        let rocket_actions =
            ts.subscribe::<RocketActions>("/rocket/actions", Capacity::Unbounded)?;
        let aero_angles = ts.subscribe::<AeroAngles>("/rocket/aero_angles", Capacity::Unbounded)?;

        Ok(RerunLogger {
            rocket_state,
            rocket_actions,
            aero_angles,
        })
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
            rcv_rocket_state: self.rocket_state,
            rcv_rocket_actions: self.rocket_actions,
            rcv_aero_angles: self.aero_angles,
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
    pub fn log(&mut self) -> Result<()> {
        let mut select = Select::default();

        select.add(&self.rcv_rocket_state);
        select.add(&self.rcv_rocket_actions);
        select.add(&self.rcv_aero_angles);

        let mut open_channels = 3;

        while open_channels > 0 {
            let ix = select.ready();

            match ix {
                0 => {
                    if let Ok(Timestamped::<RocketState>(ts, state)) = self.rcv_rocket_state.recv()
                    {
                        Self::log_rocket_state(&mut self.rec, &mut self.memory, ts, state)?;
                    } else {
                        open_channels -= 1;
                    }
                }
                1 => {
                    if let Ok(Timestamped::<RocketActions>(ts, state)) =
                        self.rcv_rocket_actions.recv()
                    {
                        // Self::log_rocket_state(&mut self.rec, &mut self.memory, ts, state)?;
                    } else {
                        open_channels -= 1;
                    }
                }
                2 => {
                    if let Ok(Timestamped::<AeroAngles>(ts, state)) = self.rcv_aero_angles.recv() {
                        // Self::log_rocket_state(&mut self.rec, &mut self.memory, ts, state)?;
                    } else {
                        open_channels -= 1;
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

        // Keep history of 3D position to display a 3D trajectory in Rerun
        memory
            .trajectory_ned_3d
            .push([pos[0] as f32, pos[1] as f32, pos[2] as f32]);

        // Keep history of latitude and longitude to display trajectory over a map
        let (lat, lon, _) = ned2geodetic(
            pos[0],
            pos[1],
            pos[2],
            origin[0],
            origin[1],
            origin[2],
            map_3d::Ellipsoid::WGS84,
        );

        memory
            .trajectory_geodetic
            .push([lat.to_degrees(), lon.to_degrees()]);

        rec.set_time_seconds("sim_time", ts.monotonic.elapsed_seconds_f64());

        rec.log(
            "/frame/body_centered",
            &rerun::Transform3D::from_translation(memory.trajectory_ned_3d.last().unwrap()),
        )?;

        // Position
        rec.log("timeseries/position/x", &rerun::Scalar::new(pos[0]))?;
        rec.log("timeseries/position/y", &rerun::Scalar::new(pos[1]))?;
        rec.log("timeseries/position/z", &rerun::Scalar::new(pos[2]))?;

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

        // Trajectory lines
        rec.log(
            "trajectory/ned_3d",
            &rerun::LineStrips3D::new([memory.trajectory_ned_3d.as_slice()]),
        )?;

        rec.log(
            "trajectory/geodetic",
            &rerun::GeoLineStrings::from_lat_lon([memory.trajectory_geodetic.as_slice()]),
        )?;

        // Current location point for map plot
        let last_pos_geo = memory.trajectory_geodetic.last().unwrap();
        rec.log(
            "objects/position_geodetic",
            &rerun::GeoPoints::from_lat_lon([(last_pos_geo[0], last_pos_geo[1])])
                .with_radii([rerun::Radius::new_ui_points(10.0)])
                .with_colors([rerun::Color::from_rgb(255, 0, 0)]),
        )?;

        // Transform
        rec.log(
            "rocket",
            &rerun::Transform3D::from_translation_rotation(
                memory.trajectory_ned_3d.last().unwrap(),
                rerun::Rotation3D::Quaternion(RotationQuat(Quaternion([
                    quat.i as f32,
                    quat.j as f32,
                    quat.k as f32,
                    quat.w as f32,
                ]))),
            ),
        )?;

        todo!()
    }
}
