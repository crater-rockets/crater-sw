
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DigitalState {
    Low = 0,
    High = 1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DigitalInputState(pub DigitalState);

