use cxx;
#[cxx::bridge]
pub mod ffi {
    unsafe extern "C++" {
        include!("gtsam_build/ceres_demo/ceres_warp.h");
        type CeresWarp;
        fn new_ceres_warp() -> UniquePtr<CeresWarp>;
        fn add_residual_block(self: Pin<&mut CeresWarp>, x: f64, y: f64); // 对于不可变引用，头文件要定义成 const函数
        fn solve(self: Pin<&mut CeresWarp>); // 对于可变引用，需要使用Pin<&mut ABC>
    }
}
