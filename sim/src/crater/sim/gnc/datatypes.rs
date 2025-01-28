use nalgebra::Vector4;

#[derive(Debug, Clone, Default)]
pub struct ServoPosition {
    pub servo_positions: Vector4<f64>,
}
