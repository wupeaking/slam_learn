use cxx;
#[cxx::bridge]
pub mod ffi {
    unsafe extern "C++" {
        include!("gtsam_build/src/pose_graph_optimization/ceres_pose_graph/pose.h");
        type POSEOPT;
        fn new_pose_opt(pose_num: i32) -> UniquePtr<POSEOPT>;
        fn add_residual_block(
            self: Pin<&mut POSEOPT>,
            pose_ij: &[f64],
            info_matrix: &[f64],
            pose_i: i32,
            pose_j: i32,
        );

        fn set_angle_axis(self: Pin<&mut POSEOPT>, pose_index: i32, x: f64, y: f64, z: f64);
        fn set_translate(self: Pin<&mut POSEOPT>, pose_index: i32, x: f64, y: f64, z: f64);
        fn solve(self: Pin<&mut POSEOPT>); // 对于可变引用，需要使用Pin<&mut ABC>
        fn get_pose(self: Pin<&mut POSEOPT>, camera_index: i32) -> Vec<f64>;
    }
}
