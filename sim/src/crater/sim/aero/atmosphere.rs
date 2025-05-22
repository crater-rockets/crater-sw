pub trait Atmosphere {
    fn pressure_pa(&self, alt_m: f64) -> f64;
    fn density_kg_m3(&self, alt_m: f64) -> f64;
    fn temperature_k(&self, alt_m: f64) -> f64;
    fn speed_of_sound_m_s(&self, alt_m: f64) -> f64;

    fn properties(&self, altitude_m: f64) -> AtmosphereProperties {
        AtmosphereProperties {
            pressure_pa: self.pressure_pa(altitude_m),
            air_density_kg_m3: self.density_kg_m3(altitude_m),
            temperature_k: self.temperature_k(altitude_m),
            speed_of_sound_m_s: self.speed_of_sound_m_s(altitude_m),
        }
    }
}
pub fn mach_number(v_air_norm_m_s: f64, c: f64) -> f64 {
    v_air_norm_m_s / c
}

#[derive(Debug, Clone)]
pub struct AtmosphereProperties {
    pub pressure_pa: f64,
    pub air_density_kg_m3: f64,
    pub temperature_k: f64,
    pub speed_of_sound_m_s: f64,
}

#[derive(Debug, Clone)]
pub struct AtmosphereIsa {
    pressure_0: f64,
    temperature_0: f64,
    density_0: f64,
    alt_0: f64,
    g_0: f64,
    specific_gas_constant: f64,
    a: f64,
}

impl Default for AtmosphereIsa {
    fn default() -> Self {
        AtmosphereIsa {
            pressure_0: 101325.0,
            temperature_0: 288.15,
            density_0: 1.2250,
            alt_0: 0.0,
            g_0: 9.80665,
            specific_gas_constant: 287.052874,
            a: -0.0065,
        }
    }
}

impl AtmosphereIsa {
    pub fn new(
        pressure_0: f64,
        temperature_0: f64,
        density_0: f64,
        alt_0: f64,
        g_0: f64,
        molar_gas_constant: f64,
        a: f64,
    ) -> AtmosphereIsa {
        AtmosphereIsa {
            pressure_0,
            temperature_0,
            density_0,
            alt_0,
            g_0,
            specific_gas_constant: molar_gas_constant,
            a,
        }
    }
}

impl Atmosphere for AtmosphereIsa {
    fn pressure_pa(&self, alt: f64) -> f64 {
        let exponent = -self.g_0 / (self.a * self.specific_gas_constant);
        let t = self.temperature_k(alt);
        (t / self.temperature_0).powf(exponent) * self.pressure_0
    }

    fn temperature_k(&self, alt: f64) -> f64 {
        self.temperature_0 + self.a * (alt - self.alt_0)
    }

    fn density_kg_m3(&self, alt: f64) -> f64 {
        let exponent = -(self.g_0 / (self.a * self.specific_gas_constant) + 1.0);
        let t = self.temperature_k(alt);
        (t / self.temperature_0).powf(exponent) * self.density_0
    }

    fn speed_of_sound_m_s(&self, alt_m: f64) -> f64 {
        f64::sqrt(1.4 * self.pressure_pa(alt_m) / self.density_kg_m3(alt_m))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_default_isa_temperature() {
        let isa = AtmosphereIsa::default();

        assert_relative_eq!(isa.temperature(0.0), 288.15, epsilon = 0.01);
        assert_relative_eq!(isa.temperature(304.8), 286.17, epsilon = 0.01);
        assert_relative_eq!(isa.temperature(1219.2), 280.23, epsilon = 0.01);
        assert_relative_eq!(isa.temperature(4572.0), 258.43, epsilon = 0.01);
        assert_relative_eq!(isa.temperature(10668.0), 218.81, epsilon = 0.01);
    }

    #[test]
    fn test_default_isa_pressure() {
        let isa = AtmosphereIsa::default();

        assert_relative_eq!(isa.pressure(0.0), 101325.0, epsilon = 1.0);
        assert_relative_eq!(isa.pressure(304.8), 97717.0, epsilon = 1.0);
        assert_relative_eq!(isa.pressure(1219.2), 87511.0, epsilon = 1.0);
        assert_relative_eq!(isa.pressure(4572.0), 57182.0, epsilon = 1.0);
        assert_relative_eq!(isa.pressure(10668.0), 23842.0, epsilon = 1.0);
    }

    #[test]
    fn test_default_isa_density() {
        let isa = AtmosphereIsa::default();

        assert_relative_eq!(isa.density(0.0), 1.2250, epsilon = 0.0001);
        assert_relative_eq!(isa.density(304.8), 1.1896, epsilon = 0.0001);
        assert_relative_eq!(isa.density(1219.2), 1.0879, epsilon = 0.0001);
        assert_relative_eq!(isa.density(4572.0), 0.7708, epsilon = 0.0001);
        assert_relative_eq!(isa.density(10668.0), 0.3796, epsilon = 0.0001);
    }
}
