extern crate bindgen;
extern crate cxx_build;

use std::env;
use std::path::PathBuf;

fn main() {
    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    let new_out_path = PathBuf::from("./src/");
    // 1. 首先将gtsam_build.cc编译为 gtsam_dynamic.so 文件
    // cc::Build::new()
    //     .file("gtsam_build_demo/gtsam_build.cc")
    //     .include("gtsam_build_demo")
    //     .cpp(true)
    //     .flag("-std=c++11")
    //     .shared_flag(true)
    //     .static_flag(false)
    //     .flag("-shared")
    //     .flag("-fPIC")
    //     .include("/usr/local/include/gtsam/3rdparty/Eigen")
    //     .compile("gtsam_dynamic");

    // std::fs::copy(
    //     out_path.join("libgtsam_dynamic.a"),
    //     new_out_path.join("libgtsam_dynamic.so"),
    // )
    // .unwrap();
    // // Tell cargo to look for shared libraries in the specified directory
    // println!("cargo:rustc-link-lib=gtsam");
    // println!("cargo:rustc-link-lib=boost_serialization");
    // //  Add a directory to the library search path
    // println!("cargo:rustc-link-search=/usr/local/lib");

    // // Tell cargo to invalidate the built crate whenever the wrapper changes
    // println!("cargo:rerun-if-changed=src/gtsam_build_demo/gtsam_build.h");

    // 利用bingen生成bindings.rs文件 并将其移动到src目录下
    let bindings = bindgen::Builder::default()
        // The input header we would like to generate
        // bindings for.
        .header("gtsam_build_demo/gtsam_build.h")
        // Tell cargo to invalidate the built crate whenever any of the
        // included header files changed.
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        // Finish the builder and generate the bindings.
        .generate()
        // Unwrap the Result and panic on failure.
        .expect("Unable to generate bindings");

    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
    // 将生成的bindings.rs文件移动到src目录下
    std::fs::copy(
        out_path.join("bindings.rs"),
        new_out_path.join("bindings.rs"),
    )
    .unwrap();

    // println!("cargo:rustc-link-lib=gtsam");
    // println!("cargo:rustc-link-lib=boost_serialization");
    // //  Add a directory to the library search path
    // println!("cargo:rustc-link-search=/usr/local/lib");
    // println!("cargo:rustc-link-lib=dylib=gtsam_dynamic");
    use cmake::Config;

    let dst = Config::new("./gtsam_build_demo/").build();
    // println!("cargo:rustc-link-lib=dylib=gtsam_dynamic");
    println!("cargo:rustc-link-arg=-lgtsam_dynamic");
    println!(
        "cargo:rustc-link-search=native={}",
        dst.join("lib").display()
    );
    std::fs::copy(
        dst.join("lib/libgtsam_dynamic.so"),
        new_out_path.join("libgtsam_dynamic.so"),
    )
    .unwrap();
    // println!("cargo:rustc-link-search=native=./gtsam_build_demo/");
}
