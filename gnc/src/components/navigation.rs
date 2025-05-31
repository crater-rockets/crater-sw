use alloc::boxed::Box;
use nalgebra::{UnitQuaternion, Vector3};
use statig::prelude::*;

use crate::{
    common::Timestamped,
    component::{Component, LoopContext},
    datatypes::{
        gnc::NavigationOutput,
        sensors::{GpsSensorSample, ImuSensorSample, MagnetometerSensorSample},
    },
    events::Event,
    hal::channel::{Receiver, Sender},
};

pub struct NavigationHarness {
    pub rx_imu: Box<dyn Receiver<ImuSensorSample> + Send>,
    pub rx_magn: Box<dyn Receiver<MagnetometerSensorSample> + Send>,
    pub rx_gps: Box<dyn Receiver<GpsSensorSample> + Send>,

    /// Used for debugging, just propagates ideal navigation output to tx_nav_out
    pub rx_mock_nav_out: Option<Box<dyn Receiver<NavigationOutput> + Send>>,
    pub tx_nav_out: Box<dyn Sender<NavigationOutput> + Send>,
}

pub struct NavigationComponent {
    state_machine: StateMachine<NavigationStateMachine>,
}

impl NavigationComponent {
    pub fn new(harness: NavigationHarness) -> Self {
        Self {
            state_machine: NavigationStateMachine::new(harness).state_machine(),
        }
    }
}

impl Component for NavigationComponent {
    fn id(&self) -> crate::mav_crater::ComponentId {
        crate::mav_crater::ComponentId::Navigation
    }

    fn handle_event(&mut self, event: Event, context: &mut LoopContext) {
        self.state_machine.handle_with_context(&event, context);
    }

    fn step(&mut self, context: &mut LoopContext) {
        self.state_machine
            .handle_with_context(&Event::Step, context);
    }
}

struct NavigationStateMachine {
    nav: NavigationAlgorithm,
}

impl NavigationStateMachine {
    fn new(harness: NavigationHarness) -> Self {
        Self {
            nav: NavigationAlgorithm::new(harness),
        }
    }
}

#[state_machine(initial = "State::idle()")]
impl NavigationStateMachine {
    #[state]
    fn idle(&mut self, event: &Event, context: &mut LoopContext) -> Response<State> {
        match event {
            Event::Step => {
                self.nav.update(context.step().step_time);
                Handled
            }
            _ => Super,
        }
    }

    #[state]
    fn calibrating(&mut self, event: &Event, context: &mut LoopContext) -> Response<State> {
        match event {
            Event::Step => {
                self.nav.update(context.step().step_time);
                Handled
            }
            _ => Super,
        }
    }

    #[state]
    fn on_pad(&mut self, event: &Event, context: &mut LoopContext) -> Response<State> {
        match event {
            Event::Step => {
                self.nav.update(context.step().step_time);
                Handled
            }
            _ => Super,
        }
    }

    #[state]
    fn flying(&mut self, event: &Event, context: &mut LoopContext) -> Response<State> {
        match event {
            Event::Step => {
                self.nav.update(context.step().step_time);
                Handled
            }
            _ => Super,
        }
    }
}

struct NavigationAlgorithm {
    harness: NavigationHarness,
}

impl NavigationAlgorithm {
    fn new(harness: NavigationHarness) -> Self {
        Self { harness }
    }

    fn update(&mut self, ts: crate::Instant) {
        while let Some(Timestamped { t, v }) = self.harness.rx_imu.try_recv() {
            // Multiple or no imu samples may have been received this step
        }

        while let Some(Timestamped { t, v }) = self.harness.rx_magn.try_recv() {
            // Multiple or no magnetometer samples may have been received this step
        }

        while let Some(Timestamped { t, v }) = self.harness.rx_gps.try_recv() {
            // Multiple or no gps samples may have been received this step
        }

        // Propagate
        let quat_bn = UnitQuaternion::<f32>::identity();
        let pos_n_m: Vector3<f32> = Vector3::<f32>::zeros();
        let vel_n_m_s: Vector3<f32> = Vector3::<f32>::zeros();
        let angvel_unbias_b_rad_s: Vector3<f32> = Vector3::<f32>::zeros();
        let acc_unbias_b_m_s2: Vector3<f32> = Vector3::<f32>::zeros();

        let nav_out = NavigationOutput {
            quat_nb: quat_bn,
            pos_n_m,
            vel_n_m_s,
            angvel_unbias_b_rad_s,
            acc_unbias_b_m_s2,
        };

        if let Some(rx_nav_out) = &mut self.harness.rx_mock_nav_out {
            if let Some(nav_out) = rx_nav_out.try_recv_last() {
                self.harness.tx_nav_out.send_immediate(nav_out.t, nav_out.v);
            }
        } else {
            self.harness.tx_nav_out.send_immediate(ts, nav_out);
        }
    }
}
