use crossbeam_channel::{Receiver, RecvError, Select};

pub struct Selector<'a> {
    select: Select<'a>,
    callbacks: Vec<Box<dyn FnMut()+ 'a>>,
}

impl<'a> Selector<'a> {
    pub fn new() -> Self {
        Self {
            select: Select::new(),
            callbacks: Vec::new()
        }
    }
    
    pub fn recv<T, F: FnMut(Result<T, RecvError>) + 'a>(
        mut self,
        receiver: &'a Receiver<T>,
        mut callback: F,
    ) -> Self {
        self.select.recv(receiver);

        let rx_fn = move || {
            let rx = receiver.recv();
            callback(rx);
        };

        self.callbacks.push(Box::new(rx_fn));

        self
    }

    pub fn ready(mut self) {
        let index = self.select.ready();

        self.callbacks[index]();
    }
}
