use std::env;
use std::path::PathBuf;

fn main() {
    // let root_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap()).join("..");
    // let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());

    // let crater_lib_dir = root_dir.join("build/ffi/fsw/shared");
    // println!("cargo:rustc-link-lib=stdc++");
    // println!("cargo:rustc-link-search={}", crater_lib_dir.display());
    // println!("cargo:rustc-link-lib=static=crater_shared");
    // let crater_bind = bindgen::Builder::default()
    //     .header(format!(
    //         "{}/fsw/shared/inc/crater/main/ffi/CraterFFI.hpp",
    //         root_dir.display()
    //     ))
    //     .clang_arg("-xc++")
    //     .clang_arg("-std=c++17")
    //     .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
    //     .generate()
    //     .expect("Unable to generate bindings");

    // crater_bind
    //     .write_to_file(out_path.join("crater_ffi.rs"))
    //     .expect("Couldn't write bindings!");

    // let mavlink_def_dir = root_dir.join("fsw/proto/message_definitions/v1.0");
    // let mavlink_bind = mavlink_bindgen::generate(mavlink_def_dir, out_path).expect("Error creating mavlink bindings");
    // mavlink_bindgen::format_generated_code(&mavlink_bind);
    // mavlink_bindgen::emit_cargo_build_messages(&mavlink_bind);
}
