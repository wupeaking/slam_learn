use cxx;
#[cxx::bridge]
pub mod ffi {
    unsafe extern "C++" {
        include!("gtsam_build/ceres_demo/ceres_ba.h");
        type CeresBA;
        fn new_ceres_ba() -> UniquePtr<CeresBA>;
        fn add_residual_block(self: Pin<&mut CeresBA>, u: f64, v: f64, x: f64, y: f64, z: f64);
        fn solve(self: Pin<&mut CeresBA>); // 对于可变引用，需要使用Pin<&mut ABC>
        fn get_angle_axis(self: Pin<&mut CeresBA>) -> Vec<f64>;
        fn get_translate(self: Pin<&mut CeresBA>) -> Vec<f64>;
    }
}
