// use crate::bindings::example;
// mod bindings;
mod ceres_bind;
mod cxx_build;
use gtsam_build::pose_estimation::*;
use opencv::{self, core, highgui, imgcodecs, imgproc, prelude::*};

fn main() {
    // use crate::demo;
    use cxx_build::ffi::ABC;
    unsafe {
        let mut c = cxx_build::ffi::new_abc();
        c.print();
        c.as_mut().unwrap().add("heee");
        println!("get 0: {} ", c.as_mut().unwrap().get(0));
    }
    // cxx_build::ffi::example();
    println!("rust call c++");

    unsafe {
        let mut ceres = ceres_bind::ffi::new_ceres_warp();
        use rand_distr::{Distribution, Normal, NormalInverseGaussian};
        let (a, b, c) = (1.0, 2.0, 1.0);
        for i in 0..100 {
            let normal = Normal::new(0.0, 2.0).unwrap();
            let v = normal.sample(&mut rand::thread_rng());
            println!("v: {}", v);
            let x = i as f64 / 100.0;
            let y = (a * x * x + b * x + c).exp() + v;
            println!("x: {}, y: {}", x, y);
            ceres.as_mut().unwrap().add_residual_block(x, y);
        }
        ceres.as_mut().unwrap().solve();
    }

    let mut k = opencv::core::Mat::new_rows_cols_with_default(
        3,
        3,
        opencv::core::CV_32FC1,
        core::Scalar_::<f64>::from([0.0, 0.0, 0.0, 0.0]),
    )
    .unwrap();
    let kk = opencv::core::Mat::from_slice_rows_cols(
        &[520.9_f32, 0.0, 325.1, 0.0, 521.0, 249.7, 0.0, 0.0, 1.0],
        3,
        3,
    )
    .unwrap();
    // kk.to_vec_2d()
    // 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1)
    *k.at_2d_mut::<f32>(0, 0).unwrap() = 520.9;
    *k.at_2d_mut::<f32>(0, 1).unwrap() = 0.0;
    *k.at_2d_mut::<f32>(0, 2).unwrap() = 325.1;
    *k.at_2d_mut::<f32>(1, 0).unwrap() = 0.0;
    *k.at_2d_mut::<f32>(1, 1).unwrap() = 521.0;
    *k.at_2d_mut::<f32>(1, 2).unwrap() = 249.7;
    *k.at_2d_mut::<f32>(2, 0).unwrap() = 0.0;
    *k.at_2d_mut::<f32>(2, 1).unwrap() = 0.0;
    *k.at_2d_mut::<f32>(2, 2).unwrap() = 1.0;
    let mut pose_estimation = PoseEstimation::new(
        kk,
        "data/1.png".to_string(),
        "data/2.png".to_string(),
        "data/1_depth.png".to_string(),
    );
    // 查找匹配点
    pose_estimation.find_match_keypoints();
    pose_estimation.solve_pnp();
    pose_estimation.ba_slove();
    // pose_estimation.draw_matches();
}
