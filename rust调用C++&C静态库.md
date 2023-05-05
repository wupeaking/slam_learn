
### 问题
目前对于调用C++的动态库 只能先用C封装一下 然后手动生成C的动态库  在build.rs里面再手动指定动态库的路径和名字
    //! 目前经过试验发现 cxx_build 最后调用的也是cc这个crate进行编译 但是这个
    // 破玩意有个问题是根本不能编译成动态库 但是他却在文档里面写支持编译动态库
    // https://github.com/rust-lang/cc-rs/issues/250
    // 特别傻逼 目前看来唯一的方案 就是先把c++的代码用c进行一个封装
    // 然后使用binggen 对c的头文件进行自动转换成bind.rs的代码
    // 把C封装的代码手动生成动态库 在build.rs里面指定动态库的路径和名称
    // 注意需要使用的是cargo:rustc-link-arg 而不是cargo:rustc-link-lib

### 步骤
1. 如何用bingen将.h文件转换成rs文件
```rust
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
```

2. 手动编译C++代码生成动态库

3. 在build.rs里面指定动态库的路径和名称
```rust
    println!("cargo:rustc-link-lib=dylib=gtsam_dynamic");
    println!("cargo:rustc-link-arg=-lgtsam_dynamic");
    println!("cargo:rustc-link-search=native=./gtsam_build_demo/");
```


cxx只能编译成静态库 所以如果rust调用的c++有静态库或者源码的话 可以用这个方法 然后指定需要链接的每个静态库
比如调用Ceres

首先安装ceres
```shell
# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev
# Use ATLAS for BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse (optional)
sudo apt-get install libsuitesparse-dev
tar zxf ceres-solver-2.1.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-2.1.0
make -j3
make test
# Optionally install Ceres, it can also be exported using CMake which
# allows Ceres to be used without requiring installation, see the documentation
# for the EXPORT_BUILD_DIR option for more information.
make install
```

2. 编写C++代码
 查看 ceres_demo/ceres_demo.cpp
3. 编写ceres_build.rs
4. 编写build.rs
```rust
    cxx_build::bridge("src/ceres_bind.rs")
        .file("ceres_demo/ceres_warp.cc")
        .cpp(true)
        .flag_if_supported("-std=c++14")
        .flag("-g")
        .include("/usr/local/include/")
        .include("/usr/local/include/gtsam/3rdparty/Eigen")
        .compile("ceres_warp");
    println!("cargo:rerun-if-changed=ceres_demo/ceres_warp.cc");
    println!("cargo:rerun-if-changed=ceres_demo/ceres_warp.h");
    println!("cargo:rustc-link-lib=ceres"); //ceres 需要依赖 glog lapack blas
    println!("cargo:rustc-link-lib=glog");
    println!("cargo:rustc-link-lib=lapack");
    println!("cargo:rustc-link-lib=blas");
```