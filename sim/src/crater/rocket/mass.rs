use nalgebra::{matrix, Matrix3, Vector3};

use crate::crater::engine::engine::RocketEngineMassProperties;

use super::rocket_data::RocketParams;

#[derive(Debug, Clone)]
pub struct RocketMassProperties {
    pub xcg_total_m: Vector3<f64>,

    pub mass_kg: f64,
    pub mass_dot_kg_s: f64,

    pub inertia_kgm2: Matrix3<f64>,
    pub inertia_dot_kgm2_s: Matrix3<f64>,
}

impl RocketMassProperties {
    pub fn calc_mass(mass_eng: &RocketEngineMassProperties, rocket: &RocketParams) -> RocketMassProperties{
        let mass_tot = rocket.mass_body_kg + mass_eng.mass_kg;

        let mass_dot = mass_eng.mass_dot_kg_s;

        let xcg_eng = rocket.engine_ref_pos_m + Vector3::new(mass_eng.xcg_eng_frame_m, 0.0, 0.0);

        let xcg_total = (mass_eng.mass_kg * xcg_eng + rocket.mass_body_kg * (rocket.xcg_body_m))
            / mass_tot;

        let inertia_body: Matrix3<f64> = rocket.inertia_body_b_kgm2
            + rocket.mass_body_kg
                * self::RocketMassProperties::parallel_axis_matrix(
                    xcg_total - rocket.xcg_body_m,
                );

        let parallel_axis_matrix_eng = self::RocketMassProperties::parallel_axis_matrix(xcg_total - xcg_eng);

        let inertia_eng: Matrix3<f64> = mass_eng.inertia_eng_frame_kgm2
            + mass_eng.mass_kg
                * parallel_axis_matrix_eng;

        let inertia = inertia_body + inertia_eng;

        let dist_prop_xcg: Vector3<f64> = xcg_total - xcg_eng;

        let skew_dist_prop_xcg: Matrix3<f64> = self::RocketMassProperties::skew_matrix(dist_prop_xcg);

        let skew_prop_dot_xcg =
            self::RocketMassProperties::skew_matrix(Vector3::new(mass_eng.xcg_dot_eng_frame_m, 0.0, 0.0));

        let inertia_dot = mass_eng.inertia_dot_eng_frame_kgm2
            + mass_eng.mass_kg
                * (skew_dist_prop_xcg.transpose() * skew_prop_dot_xcg
                    + skew_prop_dot_xcg.transpose() * skew_dist_prop_xcg)
                    + mass_dot * parallel_axis_matrix_eng;

        RocketMassProperties{
            xcg_total_m: xcg_total,
            mass_kg: mass_tot,
            mass_dot_kg_s: mass_dot,
            inertia_kgm2: inertia,
            inertia_dot_kgm2_s: inertia_dot
        }
    }

    pub fn skew_matrix(vec: Vector3<f64>) -> Matrix3<f64> {
        matrix![0.0, -vec.z, vec.y; 
                vec.z, 0.0, -vec.x; 
                -vec.y, vec.x, 0.0]
    }
    pub fn parallel_axis_matrix(delta_xcg: Vector3<f64>) -> Matrix3<f64> {
        let cross: Matrix3<f64> = self::RocketMassProperties::skew_matrix(delta_xcg);

        cross.transpose() * cross
    }
}