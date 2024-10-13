use super::NodeManager;
use anyhow::Result;
use std::{
    sync::mpsc::{channel, Receiver},
    thread::{self, JoinHandle},
};

pub struct ThreadedExecutor {
    handles: Vec<JoinHandle<Result<()>>>,
    fail_receiver: Receiver<()>,
}

impl ThreadedExecutor {
    pub fn run(node_mgr: NodeManager) -> ThreadedExecutor {
        let (fail_s, fail_r) = channel::<()>();
        let mut exec = ThreadedExecutor {
            handles: vec![],
            fail_receiver: fail_r,
        };

        for mut n in node_mgr.nodes.into_iter() {
            let fail_s = fail_s.clone();
            exec.handles.push(thread::spawn(move || -> Result<()> {
                loop {
                    n.step().inspect_err(|_| fail_s.send(()).unwrap())?;
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

    pub fn run_blocking(
        node_mgr: NodeManager,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        todo!()
    }
}
