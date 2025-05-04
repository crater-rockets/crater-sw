use std::env;
use std::path::PathBuf;

fn main() {
    let definitions_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap()).join("proto");

    let out_dir = env::var("OUT_DIR").unwrap();
    let mav_bindings = match mavlink_bindgen::generate(definitions_dir, out_dir) {
        Ok(r) => r,
        Err(e) => {
            panic!("Failed to generate MAVLink bindings: {}", e);
        }
    };

    mavlink_bindgen::format_generated_code(&mav_bindings);
    mavlink_bindgen::emit_cargo_build_messages(&mav_bindings);
}
