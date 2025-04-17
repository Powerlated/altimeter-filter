use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo:rustc-link-lib=static=altimeterfilter");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    let src_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src/");

    // Generate the bindings
    let mut builder = bindgen::Builder::default()
        .header(src_path.join("AltimeterFilter.h").to_string_lossy())
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .wrap_static_fns(true)
        .wrap_static_fns_path(out_path.join("extern.c"))
        .use_core();

    let bindings = builder.generate().expect("Unable to generate bindings");
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Failed to write bindings");

    // Compile the C library
    cc::Build::new()
        .files(&[
            src_path.join("AltimeterFilter.c"),
        ])
        .compile("AltimeterFilter");
}