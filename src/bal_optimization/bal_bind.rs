use cxx;
#[cxx::bridge]
pub mod ffi {
    unsafe extern "C++" {
        include!("gtsam_build/src/bal_optimization/ceres_bal/bal.h");
        type BAL;
        fn new_bal(camera_num: i32, landmark_num: i32) -> UniquePtr<BAL>;
        fn add_residual_block(
            self: Pin<&mut BAL>,
            u: f64,
            v: f64,
            camera_index: i32,
            landmark_index: i32,
        );

        fn set_angle_axis(self: Pin<&mut BAL>, camera_index: i32, x: f64, y: f64, z: f64);
        fn set_translate(self: Pin<&mut BAL>, camera_index: i32, x: f64, y: f64, z: f64);
        fn set_camera_inter(self: Pin<&mut BAL>, camera_index: i32, f: f64, k1: f64, k2: f64);
        fn set_landmark(self: Pin<&mut BAL>, landmark_index: i32, x: f64, y: f64, z: f64);
        fn solve(self: Pin<&mut BAL>); // 对于可变引用，需要使用Pin<&mut ABC>
        fn get_camera_args(self: Pin<&mut BAL>, camera_index: i32) -> Vec<f64>;
        fn get_landmark(&self, landmark_index: i32) -> Vec<f64>;
    }
}
