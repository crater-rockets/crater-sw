use crate::{crater::sim::{actuators::ideal::IdealServo, gnc::openloop::OpenloopControl, rocket::Rocket, sensors::ideal::IdealIMU}, nodes::NodeManager};
use anyhow::Result;

pub trait ModelBuilder {
    fn build(&self, node_manager: &mut NodeManager) -> Result<()>;
}

#[derive(Debug, Clone)]
pub struct OpenLoopCrater {}

impl ModelBuilder for OpenLoopCrater {
    fn build(&self, nm: &mut NodeManager) -> Result<()> {
        nm.add_node("rocket", |ctx| Ok(Box::new(Rocket::new("crater", ctx)?)))?;
        nm.add_node("openloop_control", |ctx| {
            Ok(Box::new(OpenloopControl::new(ctx)?))
        })?;
        nm.add_node("ideal_servo", |ctx| Ok(Box::new(IdealServo::new(ctx)?)))?;
        nm.add_node("ideal_imu", |ctx| Ok(Box::new(IdealIMU::new(ctx)?)))?;

        Ok(())
    }
}
