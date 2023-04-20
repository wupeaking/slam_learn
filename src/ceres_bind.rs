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

// test
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn it_works() {
        unsafe {
            let mut ceres = ffi::new_ceres_warp();
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
    }
}
