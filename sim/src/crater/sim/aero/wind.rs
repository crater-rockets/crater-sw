use hdf5_metno::dataset::FillValue::Undefined;
use crate::{
    core::time::{Clock, Timestamp},
    nodes::{Node, NodeContext, StepResult},
    telemetry::{TelemetryReceiver, TelemetrySender, Timestamped},
    utils::capacity::Capacity::Unbounded,
};
use nalgebra::Vector3;
use rand::rngs::StdRng;
use rand::{rng, Rng, SeedableRng};
use rand_distr::{Distribution, Normal};
use crate::crater::sim::rocket::rocket_data::RocketState;

#[derive(Debug)]
pub struct WindParams {
    pub seed_w: [u8; 32],
    pub seed_u: [u8; 32],
    pub seed_v: [u8; 32],
    pub random_seed: bool,
    pub rng_u: StdRng,
    pub rng_v: StdRng,
    pub rng_w: StdRng,
}

#[derive(Debug)]
pub struct WindSample {}
#[derive(Debug)]
pub struct Wind {
    rx_state: TelemetryReceiver<RocketState>,
    params: WindParams,
    tx_wind_ned: TelemetrySender<WindSample>,
    tx_wind_body: TelemetrySender<WindSample>,
}

impl Wind {
    pub fn new(ctx: NodeContext) -> Result<Self> {
        let rx_state = ctx.telemetry().subscribe("/rocket/state", Unbounded)?;

        let params = ctx.parameters().get_map("sim.rocket.wind")?;

        let tx_wind_ned = ctx.telemetry().publish("/sensors/wind/ned")?;
        let tx_wind_body = ctx.telemetry().publish("/sensors/wind/body")?;

        let mut seed_w: [u8; 32]= ctx.parameters().get_param("sim.rocket.wind.seed_w")?.value_int()?;
        let mut seed_u: [u8; 32]= ctx.parameters().get_param("sim.rocket.wind.seed_w")?.value_int()?;
        let mut seed_v: [u8; 32]= ctx.parameters().get_param("sim.rocket.wind.seed_w")?.value_int()?;

        let random_seed: bool = ctx.parameters().get_param("sim.rocket.random_seed")?.value_bool()?;

        if random_seed {
            seed_w = [0; 32];
            rng().fill(&mut seed_w);
            seed_u = [0; 32];
            rng().fill(&mut seed_u);
            seed_v = [0; 32];
            rng().fill(&mut seed_v);
        }

        let rng_w: StdRng = StdRng::from_seed(seed_w);
        let rng_u: StdRng = StdRng::from_seed(seed_u);
        let rng_v: StdRng = StdRng::from_seed(seed_v);

        let wind_params = WindParams{
            seed_w,
            seed_u,
            seed_v,
            random_seed,
            rng_w, 
            rng_u,
            rng_v,
        };

        Ok(Self{
            rx_state,
            params: wind_params,
            tx_wind_ned,
            tx_wind_body,
        })
    }
    
}