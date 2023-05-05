// extern crate pkg_config;
fn main() {
    //! 目前经过试验发现 cxx_build 最后调用的也是cc这个crate进行编译 但是这个
    // 破玩意有个问题是根本不能编译成动态库 但是他却在文档里面写支持编译动态库
    // https://github.com/rust-lang/cc-rs/issues/250
    // 特别傻逼 目前看来唯一的方案 就是先把c++的代码用c进行一个封装
    // 然后使用binggen 对c的头文件进行自动转换成bind.rs的代码
    // 把C封装的代码手动生成动态库 在build.rs里面指定动态库的路径和名称
    // 注意需要使用的是cargo:rustc-link-arg 而不是cargo:rustc-link-lib
    cxx_build::bridge("src/cxx_build.rs")
        .file("gtsam_build_demo/build_static.cc")
        .flag_if_supported("-std=c++11")
        .flag("-g")
        .include("/usr/local/include/")
        .include("/usr/local/include/gtsam/3rdparty/Eigen")
        // .include("/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3")
        // .include("/opt/homebrew/Cellar/boost/1.78.0_1/include")
        // .warnings(false)
        // .shared_flag(true)
        // .pic(true)
        // .flag("-shared")
        // .flag("-fPIC")
        // .out_dir("./gtsam_build_demo/")
        .compiler("g++")
        .compile("cxxbridge-demo-static");

    cxx_build::bridge("src/ceres_bind.rs")
        .file("ceres_demo/ceres_warp.cc")
        .cpp(true)
        .flag_if_supported("-std=c++14")
        .flag("-g")
        .include("/usr/local/include/")
        .include("/usr/local/include/gtsam/3rdparty/Eigen")
        .compile("ceres_warp");

    cxx_build::bridge("src/pose_estimation/ceres_ba_bind.rs")
        .file("ceres_demo/ceres_ba.cc")
        .cpp(true)
        .flag_if_supported("-std=c++14")
        .flag("-g")
        .include("/usr/local/include/")
        .include("/usr/local/include/gtsam/3rdparty/Eigen")
        .compile("ceres_ba");

    // println!("cargo:rerun-if-changed=src/main.rs");
    println!("cargo:rerun-if-changed=gtsam_build_demo/gtsam_build.cc");
    println!("cargo:rerun-if-changed=ceres_demo/ceres_warp.h");
    println!("cargo:rerun-if-changed=ceres_demo/ceres_ba.h");
    println!("cargo:rerun-if-changed=ceres_demo/ceres_ba.cc");
    println!("cargo:rustc-link-lib=ceres"); //ceres 需要依赖 glog lapack blas
    println!("cargo:rustc-link-lib=glog");
    println!("cargo:rustc-link-lib=lapack");
    println!("cargo:rustc-link-lib=blas");
    // println!("cargo:rustc-link-lib=dylib=gtsam_dynamic");
    // println!("cargo:rustc-link-arg=-Wl,--start-group");
    // println!("cargo:rustc-link-arg=-lcxxbridge-demo");
    // println!("cargo:rustc-link-arg=-lstdc++");
    // println!("cargo:rustc-link-arg=-lboost_serialization");
    // println!("cargo:rustc-link-arg=-lgtsam");
    // println!("cargo:rustc-link-search=native=./gtsam_build_demo/");
    println!("cargo:rustc-link-lib=boost_serialization");
    println!("cargo:rustc-link-lib=gtsam");
    println!("cargo:rustc-link-lib=stdc++");
    // //  Add a directory to the library search path
    println!("cargo:rustc-link-search=/usr/local/lib");
    // export DYLD_LIBRARY_PATH=/opt/homebrew/opt/boost/lib:$DYLD_LIBRARY_PATH

    cxx_build::bridge("src/bal_optimization/bal_bind.rs")
        .file("src/bal_optimization/ceres_bal/bal.cc")
        .cpp(true)
        .opt_level(3) //如果不优化 ceres会需要特别长的时间  2w多个路标一次迭代需要8s左右 优化之后只需要0.2s左右
        .flag_if_supported("-std=c++14")
        .flag("-g")
        .include("/usr/local/include/")
        .include("/usr/local/include/gtsam/3rdparty/Eigen")
        .compile("bal");
    println!("cargo:rerun-if-changed=src/bal_optimization/bal.cc");
    println!("cargo:rerun-if-changed=src/bal_optimization/bal.h");
    println!("cargo:rustc-link-lib=ceres"); //ceres 需要依赖 glog lapack blas
    println!("cargo:rustc-link-lib=glog");
    println!("cargo:rustc-link-lib=lapack");
    println!("cargo:rustc-link-lib=blas");
    println!("cargo:rustc-link-lib=boost_serialization");
    println!("cargo:rustc-link-lib=gtsam");
    println!("cargo:rustc-link-lib=stdc++");
    println!("cargo:rustc-link-search=/usr/local/lib");

    cxx_build::bridge("src/pose_graph_optimization/pose_bind.rs")
        .file("src/pose_graph_optimization/ceres_pose_graph/pose.cc")
        .cpp(true)
        .opt_level(3) //如果不优化 ceres会需要特别长的时间  2w多个路标一次迭代需要8s左右 优化之后只需要0.2s左右
        .flag_if_supported("-std=c++14")
        .flag("-g")
        .include("/usr/local/include/")
        .include("/usr/local/include/gtsam/3rdparty/Eigen")
        .compile("poseopt");
    println!("cargo:rerun-if-changed=src/pose_graph_optimization/ceres_pose_graph/pose.cc");
    println!("cargo:rerun-if-changed=src/pose_graph_optimization/ceres_pose_graph/pose.h");
    println!("cargo:rustc-link-lib=ceres"); //ceres 需要依赖 glog lapack blas
    println!("cargo:rustc-link-lib=glog");
    println!("cargo:rustc-link-lib=lapack");
    println!("cargo:rustc-link-lib=blas");
    println!("cargo:rustc-link-lib=boost_serialization");
    println!("cargo:rustc-link-lib=gtsam");
    println!("cargo:rustc-link-lib=stdc++");
    println!("cargo:rustc-link-search=/usr/local/lib");
}
