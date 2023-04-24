pub mod pose_estimation3d2d;
pub use pose_estimation3d2d::*;
pub mod pose_estimation;
pub use pose_estimation::*;
pub mod pose_estimation2d2d;

pub trait PoseEstimationDemo {
    fn run(&mut self);
}
