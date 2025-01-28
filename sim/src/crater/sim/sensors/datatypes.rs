use nalgebra::Vector3;


#[derive(Debug, Clone, Default)]
pub struct IMUSample {
    pub acc: Vector3<f64>,
    pub gyro: Vector3<f64>,
}

#[derive(Debug, Clone, Default)]
pub struct GPSSample {
    pub pos_n: Vector3<f64>,
    pub vel_n: Vector3<f64>,
}

#[derive(Debug, Clone, Default)]
pub struct MagnetometerSample {
    pub magfield_b: Vector3<f64>,
}
