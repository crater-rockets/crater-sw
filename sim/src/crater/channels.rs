pub mod sim {
    pub const SIM_EVENTS: &str = "/sim/events";
}

pub mod rocket {
    pub const STATE: &str = "/rocket/state";
    pub const ACTIONS: &str = "/rocket/actions";
    pub const ACCEL: &str = "/rocket/accel";
    pub const AERO_STATE: &str = "/rocket/aerostate";
    pub const MASS_ROCKET: &str = "/rocket/mass/rocket";
    pub const MASS_ENGINE: &str = "/rocket/mass/engine";
}

pub mod gnc {
    pub const GNC_EVENTS: &str = "/gnc/events";
    pub const ADA_OUTPUT: &str = "/gnc/ada";

    pub const NAV_OUTPUT: &str = "/gnc/nav";
    pub const SERVO_COMMAND: &str = "/gnc/contro/servo_command";
}

pub mod sensors {
    pub const LIFTOFF_PIN: &str = "/sensors/liftoff_pin";

    pub const IDEAL_STATIC_PRESSURE: &str = "/sensors/ideal/static_pressure";
    pub const STATIC_PRESSURE: &str = "/sensors/static_pressure";

    pub const IDEAL_GPS: &str = "/sensors/ideal/gps";
    pub const GPS: &str = "/sensors/gps";

    pub const IDEAL_IMU: &str = "/sensors/ideal/imu";
    pub const IDEAL_IMU_CG: &str = "/sensors/ideal/imu_cg";
    pub const IMU_REAL: &str = "/sensors/ideal/imu_real";
    pub const IMU: &str = "/sensors/imu";

    pub const IDEAL_MAGNETOMETER: &str = "/sensors/ideal/magnetometer";
    pub const MAGNETOMETER: &str = "/sensors/magnetometer";

    pub const IDEAL_NAV_OUTPUT: &str = "/sensors/ideal_nav";
}

pub mod actuators {
    pub const IDEAL_SERVO_POSITION: &str = "/actuators/ideal_servo_position";
}
