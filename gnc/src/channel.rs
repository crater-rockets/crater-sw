use thiserror::Error;

#[derive(Error, Debug)]
pub enum ReceiveError {
    #[error("No data available")]
    Empty,
}
pub trait Receiver {
    type Item;

    fn try_recv(&mut self) -> Result<Self::Item, ReceiveError>;

    fn len(&self) -> usize;

    fn capacity(&self) -> usize;

    fn is_empty(&self) -> bool;

    fn is_full(&self) -> bool;

    fn num_lagged(&self) -> usize;
}
