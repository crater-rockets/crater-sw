pub mod gnc {
    pub const EVENTS: &str = "/gnc/events";
    pub const ADA_OUTPUT: &str = "/gnc/ada";
    pub const NAV_OUTPUT: &str = "/gnc/nav";
}

pub mod sensors {
    pub const LIFTOFF_PIN: &str = "/sensors/liftoff_pin";

    pub const IDEAL_STATIC_PRESSURE: &str = "/sensors/ideal_static_pressure";
    pub const STATIC_PRESSURE: &str = IDEAL_STATIC_PRESSURE;

    pub const IDEAL_GPS: &str = "/sensors/ideal_gps";
    pub const GPS: &str = IDEAL_GPS;

    pub const IDEAL_IMU: &str = "/sensors/ideal_imu";
    pub const IDEAL_IMU_CG: &str = "/sensors/ideal_imu_cg";
    pub const IMU: &str = IDEAL_IMU;

    pub const IDEAL_MAGNETOMETER: &str = "/sensors/ideal_magnetometer";
    pub const MAGNETOMETER: &str = IDEAL_MAGNETOMETER;

    pub const IDEAL_NAV_OUTPUT: &str = "/sensors/ideal_nav";
}
