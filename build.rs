use std::io::Result;
fn main() -> Result<()> {
    prost_reflect_build::Builder::new()
        .descriptor_pool("crate::DESCRIPTOR_POOL")
        .compile_protos(
            &["proto/basic.proto", "proto/sensors.proto", "proto/examples.proto"],
            &["./"],
        )?;
        
    println!("cargo:rerun-if-changed=proto/");
    Ok(())
}
