use super::{NodeManager, StepResult};
use anyhow::{Context, Result};
use chrono::{TimeDelta, Utc};
use crater_core::time::SimulatedClock;

// pub struct ThreadedExecutor {
//     node_join_handles: HashMap<String, JoinHandle<Result<()>>>,
//     clock: Arc<SystemClock>,
// }

// impl ThreadedExecutor {
//     pub fn run(node_mgr: NodeManager) -> ThreadedExecutor {
//         let stop = Arc::new(AtomicBool::new(false));

//         let mut exec = ThreadedExecutor {
//             node_join_handles: HashMap::new(),
//             clock: Arc::new(SystemClock {}),
//         };

//         for (name, node) in node_mgr.nodes.into_iter() {
//             let stop = stop.clone();
//             let clock = exec.clock.clone();

//             exec.node_join_handles.insert(
//                 name,
//                 thread::spawn(move || -> Result<()> {
//                     Self::node_thread(node, clock.as_ref(), stop)
//                 }),
//             );
//         }

//         exec
//     }

//     fn node_thread(
//         mut node: Box<dyn Node + Send>,
//         clock: &dyn Clock,
//         stop: Arc<AtomicBool>,
//     ) -> Result<()> {
//         while !stop.load(std::sync::atomic::Ordering::Relaxed) {
//             let res = node
//                 .step(clock)
//                 .inspect_err(|_| stop.store(true, std::sync::atomic::Ordering::Relaxed))?;

//             match res {
//                 StepResult::Stop => break,
//                 StepResult::Continue => continue,
//             }
//         }
//         stop.store(true, std::sync::atomic::Ordering::Relaxed);
//         Ok(())
//     }

//     pub fn join(self) -> Result<()> {
//         let mut res = Ok(());
//         for (name, h) in self.node_join_handles {
//             if let Err(e) = h.join().unwrap() {
//                 if res.is_ok() {
//                     res =
//                         Err(e).with_context(|| format!("Node {}: step() reported an error", name));
//                 }
//             }
//         }
//         res
//     }
// }

pub struct FtlOrderedExecutor;

impl FtlOrderedExecutor {
    pub fn run_blocking(mut node_mgr: NodeManager, simulated_step_period: TimeDelta) -> Result<()> {
        let mut clock = SimulatedClock::new(Utc::now(), TimeDelta::zero());

        let mut outer_res = Ok(StepResult::Continue);
        let mut stop = false;

        let mut i = 0;
        while !stop {
            clock.step(simulated_step_period);

            for (name, node) in node_mgr.nodes.iter_mut() {
                let res = node
                    .step(i, simulated_step_period, &clock)
                    .with_context(|| format!("Node {}: step() reported an error", name));

                match res {
                    Ok(StepResult::Continue) => (),
                    Err(e) => {
                        outer_res = Err(e);
                        stop = true;
                    }
                    _ => stop = true,
                }
            }

            i += 1;
        }

        outer_res?;
        Ok(())
    }
}
