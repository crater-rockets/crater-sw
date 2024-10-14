use crate::utils::time::{SimulatedClock, WallClock};

use super::NodeManager;
use anyhow::Result;
use chrono::{TimeDelta, Utc};
use std::{
    sync::{
        mpsc::{channel, Receiver},
        Arc,
    },
    thread::{self, JoinHandle},
};

pub struct ThreadedExecutor {
    handles: Vec<JoinHandle<Result<()>>>,
    fail_receiver: Receiver<()>,
    clock: Arc<WallClock>,
}

impl ThreadedExecutor {
    pub fn run(node_mgr: NodeManager) -> ThreadedExecutor {
        let (fail_s, fail_r) = channel::<()>();
        let mut exec = ThreadedExecutor {
            handles: vec![],
            fail_receiver: fail_r,
            clock: Arc::new(WallClock {}),
        };

        for mut n in node_mgr.nodes.into_iter() {
            let fail_s = fail_s.clone();
            let clock = exec.clock.clone();
            exec.handles.push(thread::spawn(move || -> Result<()> {
                loop {
                    n.step(clock.as_ref())
                        .inspect_err(|_| fail_s.send(()).unwrap())?;
                }
            }));
        }

        exec
    }

    pub fn join(self) -> Result<()> {
        let _ = self.fail_receiver.recv();
        let mut res = Ok(());
        for h in self.handles {
            if let Err(e) = h.join().unwrap() {
                res = Err(e);
            }
        }

        res
    }
}
