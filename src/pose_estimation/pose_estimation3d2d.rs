/*!
 * 使用pnp算法估计相机位姿 然后使用BA优化
 * 使用两种方案根据3d点和2d点求解相机的位姿
 * 1. 使用opencv的solvePnP
 * 2. 使用ceres的BA优化
 *
 * 流程：
 * 1. 读取图像和深度图像
 * 2. 提取特征点和描述子
 * 3. 匹配特征点
 * 4. 使用pnp算法求解相机位姿
 * 5. 使用ceres的BA优化求解相机位姿
 */
use crate::pose_estimation::pose_estimation::PoseEstimation;
use nalgebra::{Isometry3, Matrix2, Matrix6, Vector3, Vector6};
use nalgebra::{Matrix2x6, Vector2};
use opencv::core;
// use opencv::highgui;
use opencv::imgcodecs;
// use opencv::imgproc;
use opencv::prelude::*;

use super::PoseEstimationDemo;

// 3d点到2d点转换 求出R t

impl PoseEstimation<core::Point3f, core::Point2f> {
    // 根据深度图找到对应的3d 和2d点
    fn find_3d_2d_pairs(&mut self) {
        let img_depth = imgcodecs::imread(&self.img_depth, imgcodecs::IMREAD_UNCHANGED).unwrap();
        for m in &self.good_matches {
            // keypoint1 的 x y 像素
            let kp = self.keypoint1.get(m.query_idx as usize).unwrap();
            // img_depth[kp.pt().x as usize][kp.pt().y as usize];
            // 注意 opecv中x是列y是行
            let depth = img_depth
                .at_2d::<u16>(kp.pt().y as i32, kp.pt().x as i32)
                .unwrap();
            // let depth = img_depth[kp.pt().x as usize][kp.pt().y as usize];
            if *depth == 0 {
                print!("{} {} depth is zero!!!!", kp.pt().x, kp.pt().y);
                continue;
            }
            let d = *depth as f32 / 5000.0;
            let camaera_point = self.pixel_to_camaera(&kp.pt());

            let point3d = core::Point3f::new(camaera_point.x * d, camaera_point.y * d, d);
            println!("x: {} y: {} d: {}", point3d.x, point3d.y, point3d.z);
            self.object_points.push(point3d);

            let kp2 = self.keypoint2.get(m.train_idx as usize).unwrap();
            // self.image_points.push(self.pixel_to_camaera(&kp2.pt()));
            self.image_points.push(kp2.pt()); // 像素坐标 不是相机坐标系下的坐标
        }
    }

    // pnp 求解
    pub fn solve_pnp(&mut self) {
        self.find_3d_2d_pairs();
        let mut rvec = core::Mat::default();
        let mut tvec = core::Mat::default();
        // 注意：solve_pnp 2d点传入的是像素坐标 3d点传入的是相机坐标系下的坐标
        opencv::calib3d::solve_pnp(
            &self.object_points,
            &self.image_points,
            &self.camera_matrix,
            &core::Mat::default(),
            &mut rvec,
            &mut tvec,
            false,
            opencv::calib3d::SOLVEPNP_ITERATIVE, // opencv::calib3d::SOLVEPNP_EPNP,
        )
        .unwrap();
        // 旋转向量形式，用Rodrigues公式转换为矩阵
        let mut rmat = core::Mat::default();
        opencv::calib3d::rodrigues(&rvec, &mut rmat, &mut core::Mat::default()).unwrap();
        println!("rmat: {:?}", rmat.to_vec_2d::<f64>());
        println!("tvec: {:?}", tvec.to_vec_2d::<f64>());
    }
}

impl PoseEstimation<core::Point3f, core::Point2f> {
    // 通过BA优化求解
    pub fn ba_slove(&self) {
        use super::ceres_ba_bind::ffi::*;
        use nalgebra::{Unit, Vector3};
        let mut angle_axis = Vector3::new(0.0, 0.0, 0.0);
        let mut t = Vector3::new(0.0, 0.0, 0.0);
        unsafe {
            let mut ba = new_ceres_ba();
            for i in 0..self.object_points.len() {
                let point3d = self.object_points.get(i).unwrap();
                let point2d = self.image_points.get(i).unwrap();
                ba.as_mut().unwrap().add_residual_block(
                    point2d.x.into(),
                    point2d.y.into(),
                    point3d.x.into(),
                    point3d.y.into(),
                    point3d.z.into(),
                );
            }
            ba.as_mut().unwrap().solve();
            let angle = ba.as_mut().unwrap().get_angle_axis();
            angle_axis.x = angle[0];
            angle_axis.y = angle[1];
            angle_axis.z = angle[2];
            let t_ = ba.as_mut().unwrap().get_translate();
            t.x = t_[0];
            t.y = t_[1];
            t.z = t_[2];
        }
        use nalgebra::Rotation3;
        let r = Rotation3::from_axis_angle(&Unit::new_normalize(angle_axis), angle_axis.norm());
        println!("r: {:?}", r);
        println!("t: {:?}", t);
    }

    // 手动GN求解
    pub fn gn_slove(&self) {
        // R21 t21
        // let mut pose = Isometry3::new(Vector3::new(0.0, 0.0, 0.0), Vector3::y() * 0_f32);
        let mut pose = Isometry3::identity();
        let fx = self.camera_matrix.at_2d::<f32>(0, 0).unwrap();
        let fy = self.camera_matrix.at_2d::<f32>(1, 1).unwrap();
        let cx = self.camera_matrix.at_2d::<f32>(0, 2).unwrap();
        let cy = self.camera_matrix.at_2d::<f32>(1, 2).unwrap();
        let mut last_error = 0.0;
        for iter in 0..100 {
            let mut H = Matrix6::<f32>::zeros();
            // let mut H = Matrix2::<f32>::zeros();
            let mut b = Vector6::<f32>::zeros();
            // 计算所有的误差 和雅可比
            let mut error = 0.0;
            for i in 0..self.object_points.len() {
                let point3d = self.object_points.get(i).unwrap();
                let point2d = self.image_points.get(i).unwrap();
                // 转化为第二幅图的相机坐标系下坐标 P'
                let point_in_caram = pose * Vector3::new(point3d.x, point3d.y, point3d.z);
                let inv_z = 1.0 / point_in_caram.z;
                // 将估算的相机坐标转为像素坐标
                let pixel_point = Vector2::new(
                    fx * point_in_caram.x / point_in_caram.z + cx,
                    fy * point_in_caram.y / point_in_caram.z + cy,
                );
                // 误差项为 观测像素坐标 - 估算的像素坐标
                let cur_e = Vector2::new(point2d.x, point2d.y) - pixel_point;
                error += cur_e.norm();
                // 计算雅可比
                let mut jacobian = Matrix2x6::<f32>::zeros();
                jacobian[(0, 0)] = -fx * inv_z;
                jacobian[(0, 1)] = 0.0;
                jacobian[(0, 2)] = fx * point_in_caram.x * inv_z * inv_z;
                jacobian[(0, 3)] = fx * point_in_caram.x * point_in_caram.y * inv_z * inv_z;
                jacobian[(0, 4)] = -fx - fx * point_in_caram.x * point_in_caram.x * inv_z * inv_z;
                jacobian[(0, 5)] = fx * point_in_caram.y * inv_z;
                jacobian[(1, 0)] = 0.0;
                jacobian[(1, 1)] = -fy * inv_z;
                jacobian[(1, 2)] = fy * point_in_caram.y * inv_z * inv_z;
                jacobian[(1, 3)] = fy + fy * point_in_caram.y * point_in_caram.y * inv_z * inv_z;
                jacobian[(1, 4)] = -fy * point_in_caram.x * point_in_caram.y * inv_z * inv_z;
                jacobian[(1, 5)] = -fy * point_in_caram.x * inv_z;
                H += jacobian.transpose() * jacobian;
                // H += jacobian * jacobian.transpose();
                b += -jacobian.transpose() * cur_e;
            }
            // H.ldlt().solve_mut(&mut b);
            let dx = H.lu().solve(&b).unwrap();
            // let dx = H.try_inverse().unwrap() * b;

            // 如果last_error > error 说明误差在增大 说明迭代到了极值点
            if last_error > error && iter > 0 {
                break;
            }
            use crate::lie_group;
            // 要求dx的平移在前 旋转在后
            pose = lie_group::se3::exp(dx) * pose;
            // println!(
            //     "pose r: {},
            //      pose t: {},
            //      dx: {}",
            //     pose.rotation.to_rotation_matrix(),
            //     pose.translation,
            //     dx
            // );
            println!("iter: {}, error: {}", iter, error);
            last_error = error;
            if dx.norm() < 1e-6 {
                break;
            }
        }
        println!(
            "pose r: {},
             pose t: {},
            ",
            pose.rotation.to_rotation_matrix(),
            pose.translation,
        );
    }

    pub fn se3_demo(&self) {
        let mut pose = Isometry3::new(
            Vector3::new(1.0_f32, 0.0, 0.0),
            Vector3::<f32>::new(1.0, 0.0, 1.0).normalize() * std::f32::consts::FRAC_PI_6,
        );
        // 注意使用{:?}打印的旋转矩阵是按列向量形式打印的 比较坑
        println!(
            "rotation_matrix: {:?}, t: {:?}",
            pose.rotation.to_rotation_matrix(),
            pose.translation
        );
        println!(
            "rotation_matrix: {}, t: {}",
            pose.rotation.to_rotation_matrix(),
            pose.translation
        );
        // use nalgebra::{Rotation3, Unit};
        // let axis = Vector3::<f32>::new(1.0, 0.0, 1.0);
        // let axis = Unit::new_normalize(axis);
        // let rotation_matrix = Rotation3::from_axis_angle(&axis, std::f32::consts::FRAC_PI_6);
        // println!(
        //     "rotation_matrix: {:?} (1,1): {:?}",
        //     rotation_matrix,
        //     rotation_matrix[(0, 1)],
        // );
        use crate::lie_group;
        use nalgebra::Vector6;
        let mut up = Vector6::<f32>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        // up[(0, 0)] = 1e-0_f32;
        pose = lie_group::se3::exp(up) * pose;
        println!(
            "rotation_matrix_up: {:?}, t: {:?}",
            pose.rotation.to_rotation_matrix(),
            pose.translation
        );
    }
}

impl PoseEstimationDemo for PoseEstimation<core::Point3f, core::Point2f> {
    fn run(&mut self) {
        self.find_match_keypoints();
        self.solve_pnp();
        self.gn_slove();
        self.ba_slove();
        // self.draw_matches();
    }
}

#[cfg(test)]
mod pose_test {
    // cargo test -- --nocapture
    use super::*;
    #[test]
    fn test_pose_slove() {
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
        let mut pose_estimation = PoseEstimation::<core::Point3f, core::Point2f>::new(
            kk,
            "data/1.png".to_string(),
            "data/2.png".to_string(),
            "data/1_depth.png".to_string(),
        );
        // 查找匹配点
        pose_estimation.find_match_keypoints();
        pose_estimation.solve_pnp();
        pose_estimation.ba_slove();
        pose_estimation.gn_slove();
    }
}
