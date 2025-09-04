use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo::rerun-if-changed=src/cmission_example.c");
    println!("cargo::rerun-if-changed=src/cmission_example.h");
    let crate_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let mut config: cbindgen::Config = Default::default();
    config.language = cbindgen::Language::C;
    config.cpp_compat = true;

    cbindgen::Builder::new()
        .with_crate(crate_dir)
        .with_src("src/cmission.rs")
        .with_config(config)
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file("bindings.h");

    let bindings = bindgen::Builder::default()
        .header("src/cmission_example.h")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");

    cc::Build::new()
        .file("src/cmission_example.c")
        .compile("cmission_example.o");

}
