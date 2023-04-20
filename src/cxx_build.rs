use cxx;
#[cxx::bridge]
pub mod ffi {
    // C++ types and signatures exposed to Rust.
    unsafe extern "C++" {
        include!("gtsam_build/gtsam_build_demo/gtsam_build.h");
        fn example() -> i32;
    }

    unsafe extern "C++" {
        include!("gtsam_build/gtsam_build_demo/gtsam_build.h");
        type ABC;
        fn new_abc() -> UniquePtr<ABC>;
        pub fn print(&self); // 对于不可变引用，头文件要定义成 const函数
        fn add(self: Pin<&mut ABC>, s: &str); // 对于可变引用，需要使用Pin<&mut ABC>
        fn get(&self, index: i32) -> &str;
    }
}

// test
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn it_works() {
        unsafe {
            let mut c = ffi::new_abc();
            c.print();
            c.as_mut().unwrap().add("heee");
            println!("get 0: {} ", c.as_mut().unwrap().get(0));
        }
        // cxx_build::ffi::example();
        println!("rust call c++");
    }
}
