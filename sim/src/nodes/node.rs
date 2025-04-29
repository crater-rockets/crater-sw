use chrono::TimeDelta;
use rand::rngs::OsRng;
use rand_xoshiro::{
    rand_core::{RngCore, SeedableRng},
    SplitMix64,
};
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};
use thiserror::Error;

use crate::{
    core::{path::Path, time::Clock},
    parameters::ParameterMap,
    telemetry::{
        TelemetryDispatcher, TelemetryError, TelemetryReceiver, TelemetrySender, TelemetryService,
    },
    utils::capacity::Capacity,
};

#[derive(Debug, Error)]
pub enum Error {
    #[error("Missing configuration for node {0}")]
    MissingConfig(String),

    #[error(transparent)]
    NodeInstantiation(#[from] Box<dyn std::error::Error + Send + Sync>),
}

pub enum StepResult {
    Continue,
    Stop,
}

pub trait Node {
    fn step(&mut self, i: usize, dt: TimeDelta, clock: &dyn Clock) -> anyhow::Result<StepResult>;
}

pub struct NodeManager {
    telemetry: TelemetryService,
    parameters: Arc<ParameterMap>,
    nodes: Vec<(String, Box<dyn Node + Send>)>,
    rng: Arc<Mutex<SplitMix64>>,
}

impl NodeManager {
    pub fn new(telemetry: TelemetryService, parameters: ParameterMap) -> Self {
        let seed = rand::RngCore::next_u64(&mut OsRng);

        Self::new_from_seed(telemetry, parameters, seed)
    }

    pub fn new_from_seed(telemetry: TelemetryService, parameters: ParameterMap, seed: u64) -> Self {
        NodeManager {
            telemetry,
            parameters: Arc::new(parameters),
            nodes: vec![],
            rng: Arc::new(Mutex::new(SplitMix64::seed_from_u64(seed))),
        }
    }

    pub fn add_node<F>(&mut self, name: &str, creator: F) -> Result<(), Error>
    where
        F: FnOnce(
            NodeContext,
        )
            -> Result<Box<dyn Node + Send>, Box<dyn std::error::Error + Send + Sync>>,
    {
        let context = NodeContext::new(
            NodeTelemetry::new(self.telemetry.clone(), HashMap::new(), HashMap::new()),
            self.parameters.clone(),
            self.rng.clone(),
        );

        self.nodes.push((
            name.to_string(),
            creator(context).expect(format!("Error creating node '{name}'").as_str()),
        ));

        Ok(())
    }

    pub fn nodes(&self) -> &[(String, Box<dyn Node + Send>)] {
        &self.nodes
    }

    pub fn nodes_mut(&mut self) -> &mut [(String, Box<dyn Node + Send>)] {
        &mut self.nodes
    }

    pub fn parameters(&self) -> Arc<ParameterMap> {
        self.parameters.clone()
    }
}

#[derive(Debug, Clone, Default)]
pub struct NodeConfig {
    pub tm_input_map: HashMap<String, Path>,
    pub tm_output_map: HashMap<String, Path>,
}

#[derive(Debug)]
pub struct NodeContext {
    tm_dispatcher: NodeTelemetry,
    parameters: Arc<ParameterMap>,
    rng: Arc<Mutex<SplitMix64>>,
}

impl NodeContext {
    fn new(
        tm_dispatcher: NodeTelemetry,
        parameters: Arc<ParameterMap>,
        rng: Arc<Mutex<SplitMix64>>,
    ) -> Self {
        Self {
            tm_dispatcher,
            parameters,
            rng,
        }
    }

    pub fn telemetry(&self) -> &NodeTelemetry {
        &self.tm_dispatcher
    }

    pub fn parameters(&self) -> &ParameterMap {
        &self.parameters
    }

    pub fn get_rng_256<R>(&self) -> R
    where
        R: SeedableRng<Seed = [u8; 32]>,
    {
        let mut seed: [u8; 32] = [0; 32];

        self.rng.lock().unwrap().fill_bytes(&mut seed);

        R::from_seed(seed)
    }
}

#[derive(Debug)]
pub struct NodeTelemetry {
    telemetry: TelemetryService,
    input_map: HashMap<String, Path>,
    output_map: HashMap<String, Path>,
}

impl NodeTelemetry {
    pub fn new(
        ts: TelemetryService,
        input_map: HashMap<String, Path>,
        output_map: HashMap<String, Path>,
    ) -> Self {
        NodeTelemetry {
            telemetry: ts,
            input_map,
            output_map,
        }
    }
}

impl TelemetryDispatcher for NodeTelemetry {
    fn publish<T: 'static + Send>(
        &self,
        channel_name: &str,
    ) -> Result<TelemetrySender<T>, TelemetryError> {
        let path = if self.output_map.contains_key(channel_name) {
            self.output_map.get(channel_name).unwrap().clone()
        } else {
            Path::from_str(channel_name).map_err(|_| TelemetryError::InvalidChannelName)?
        };

        self.telemetry.publish::<T>(path.as_str())
    }

    fn subscribe<T: 'static + Send>(
        &self,
        channel_name: &str,
        capacity: Capacity,
    ) -> Result<TelemetryReceiver<T>, TelemetryError> {
        let path = if self.input_map.contains_key(channel_name) {
            self.input_map.get(channel_name).unwrap().clone()
        } else {
            Path::from_str(channel_name).map_err(|_| TelemetryError::InvalidChannelName)?
        };

        self.telemetry.subscribe::<T>(path.as_str(), capacity)
    }
}
