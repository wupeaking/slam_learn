// use crate::bindings::example;
// mod bindings;
mod ceres_bind;
mod cxx_build;
use gtsam_build::pose_estimation::*;
use opencv::{self, core, highgui, imgcodecs, imgproc, prelude::*};

fn main() {
    // use crate::demo;

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
    let mut pose_estimation = PoseEstimation::<core::Point2f, core::Point2f>::new(
        kk,
        "data/1.png".to_string(),
        "data/2.png".to_string(),
        "data/1_depth.png".to_string(),
    );
    // 查找匹配点
    pose_estimation.find_match_keypoints();
    // pose_estimation.solve_pnp();
    let e_mat = pose_estimation.solve_carame_pose();
    pose_estimation.check_epipolar_constraint(&e_mat);
    // pose_estimation.ba_slove();
    // pose_estimation.gn_slove();
    pose_estimation.draw_matches();
}
