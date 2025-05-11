use crate::{
    crater::sim::{
        gnc::{fsw::FlightSoftware, orchestrator::Orchestrator},
        rocket::Rocket,
        sensors::ideal::{IdealIMU, IdealMagnetometer, IdealStaticPressureSensor},
    },
    nodes::NodeManager,
};
use anyhow::Result;

pub trait ModelBuilder {
    fn build(&self, node_manager: &mut NodeManager) -> Result<()>;
}

#[derive(Debug, Clone)]
pub struct OpenLoopCrater {}

impl ModelBuilder for OpenLoopCrater {
    fn build(&self, nm: &mut NodeManager) -> Result<()> {
        nm.add_node("orchestrator", |ctx| Ok(Box::new(Orchestrator::new(ctx)?)))?;
        nm.add_node("rocket", |ctx| Ok(Box::new(Rocket::new("crater", ctx)?)))?;
        nm.add_node("ideal_imu", |ctx| Ok(Box::new(IdealIMU::new(ctx)?)))?;
        nm.add_node("ideal_mag", |ctx| {
            Ok(Box::new(IdealMagnetometer::new(ctx)?))
        })?;
        nm.add_node("ideal_press", |ctx| {
            Ok(Box::new(IdealStaticPressureSensor::new(ctx)?))
        })?;

        nm.add_node("fsw", |ctx| Ok(Box::new(FlightSoftware::new(ctx)?)))?;

        Ok(())
    }
}
