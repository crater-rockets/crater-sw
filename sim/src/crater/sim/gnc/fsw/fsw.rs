use chrono::TimeDelta;
use crater_gnc::{
    DurationU64, InstantU64,
    component::StepData,
    components::{ada::AdaHarness, fmm::FmmHarness, navigation::NavigationHarness},
    events::{EventItem, EventPublisher, EventQueue},
    gnc_main::{CraterLoop, CraterLoopHarness},
    mav_crater::ComponentId,
};

use crate::{
    core::time::Clock,
    crater::channels,
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryReceiver, Timestamped},
    utils::capacity::Capacity,
};
use anyhow::Result;

pub struct FlightSoftware {
    crater: CraterLoop,
    rx_gnc_events: TelemetryReceiver<EventItem>,
    ev_pub: EventPublisher,
}

impl FlightSoftware {
    pub fn new(ctx: NodeContext) -> Result<Self> {
        let harness = CraterLoopHarness {
            tx_events: Box::new(ctx.telemetry().publish_mp(channels::gnc::GNC_EVENTS)?),
            fmm: FmmHarness {
                rx_liftoff_pin: Box::new(
                    ctx.telemetry()
                        .subscribe(channels::sensors::LIFTOFF_PIN, Capacity::Unbounded)?,
                ),
            },
            ada: AdaHarness {
                rx_static_pressure: Box::new(ctx.telemetry().subscribe(
                    channels::sensors::IDEAL_STATIC_PRESSURE,
                    Capacity::Unbounded,
                )?),
                tx_ada_data: Box::new(ctx.telemetry().publish(channels::gnc::ADA_OUTPUT)?),
            },
            nav: NavigationHarness {
                rx_gps: Box::new(
                    ctx.telemetry()
                        .subscribe(channels::sensors::IDEAL_GPS, Capacity::Unbounded)?,
                ),
                rx_imu: Box::new(
                    ctx.telemetry()
                        .subscribe(channels::sensors::IDEAL_IMU, Capacity::Unbounded)?,
                ),
                rx_magn: Box::new(
                    ctx.telemetry()
                        .subscribe(channels::sensors::IDEAL_MAGNETOMETER, Capacity::Unbounded)?,
                ),
                rx_mock_nav_out: Some(Box::new(
                    ctx.telemetry()
                        .subscribe(channels::sensors::IDEAL_NAV_OUTPUT, Capacity::Unbounded)?,
                )),

                tx_nav_out: Box::new(ctx.telemetry().publish(channels::gnc::NAV_OUTPUT)?),
            },
        };

        let event_queue = EventQueue::default();
        let ev_pub = event_queue.get_publisher(ComponentId::Ground);
        let rx_gnc_events = ctx
            .telemetry()
            .subscribe_mp(channels::gnc::GNC_EVENTS, Capacity::Unbounded)?;

        Ok(Self {
            crater: CraterLoop::new(event_queue, harness)?,
            ev_pub,
            rx_gnc_events,
        })
    }
}

impl Node for FlightSoftware {
    fn step(&mut self, i: usize, dt: TimeDelta, clock: &dyn Clock) -> Result<StepResult> {
        while let Ok(Timestamped(_, ev)) = self.rx_gnc_events.try_recv() {
            if ev.src == ComponentId::Ground {
                self.ev_pub.publish(
                    ev.event,
                    InstantU64::from_ticks(dt.num_microseconds().unwrap() as u64).into(),
                );
            }
        }

        self.crater.step(&StepData {
            step_time: InstantU64::from_ticks(
                clock.monotonic().elapsed().num_microseconds().unwrap() as u64,
            )
            .into(),
            step_interval: DurationU64::micros(dt.num_microseconds().unwrap() as u64).into(),
            step_count: i as u32,
        });

        Ok(StepResult::Continue)
    }
}
