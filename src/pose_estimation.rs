/*!
 * 使用pnp算法估计相机位姿 然后使用BA优化
 */
use nalgebra::{Isometry3, Matrix6, Vector3, Vector6};
use nalgebra::{Matrix2x6, Vector2};
use opencv::core;
use opencv::highgui;
use opencv::imgcodecs;
use opencv::imgproc;
use opencv::prelude::*;

// 3d点到2d点转换 求出R t
pub struct PoseEstimation {
    // 相机内参
    camera_matrix: core::Mat,
    // 3D点
    object_points: core::Vector<core::Point3f>,
    // 2D点
    image_points: core::Vector<core::Point2f>,
    // 匹配的特征点
    good_matches: core::Vector<core::DMatch>,

    keypoint1: core::Vector<core::KeyPoint>,
    keypoint2: core::Vector<core::KeyPoint>,
    descriptors1: core::Mat,
    descriptors2: core::Mat,
    img1: String,
    img2: String,
    img_depth: String,
}

impl PoseEstimation {
    pub fn new(camera_matrix: core::Mat, img1: String, img2: String, img_depth: String) -> Self {
        let object_points = core::Vector::<core::Point3f>::new();
        let image_points = core::Vector::<core::Point2f>::new();
        let good_matches = core::Vector::<core::DMatch>::new();
        Self {
            camera_matrix,
            object_points,
            image_points,
            good_matches,
            keypoint1: core::Vector::<core::KeyPoint>::new(),
            keypoint2: core::Vector::<core::KeyPoint>::new(),
            descriptors1: core::Mat::default(),
            descriptors2: core::Mat::default(),
            img1,
            img2,
            img_depth,
        }
    }

    pub fn find_match_keypoints(&mut self) {
        let mut img1 = imgcodecs::imread(&self.img1, imgcodecs::IMREAD_COLOR).unwrap();
        let mut img2 = imgcodecs::imread(&self.img2, imgcodecs::IMREAD_COLOR).unwrap();
        let mut keypoints1 = core::Vector::<core::KeyPoint>::new();
        let mut keypoints2 = core::Vector::<core::KeyPoint>::new();
        let mut descriptors1 = core::Mat::default();
        let mut descriptors2 = core::Mat::default();
        self.detect_keypoint_desc(&mut img1, &mut keypoints1, &mut descriptors1);
        self.detect_keypoint_desc(&mut img2, &mut keypoints2, &mut descriptors2);

        // //  匹配特征点
        let mut matches = core::Vector::<core::DMatch>::new();
        let mut matcher =
            <dyn opencv::features2d::DescriptorMatcher>::create("BruteForce-Hamming").unwrap();
        matcher.add(&descriptors2).unwrap();
        matcher
            .match_(&descriptors1, &mut matches, &core::Mat::default())
            .unwrap();
        println!(
            "matches size: {}, origin size: {}",
            matches.len(),
            descriptors1.rows()
        );
        let (mut min_dist, mut max_dist) = (f32::MAX, 0.0);
        matches.iter().for_each(|m| {
            if min_dist > m.distance {
                min_dist = m.distance;
            }
            if max_dist < m.distance {
                max_dist = m.distance;
            }
        });
        println!("min_dist: {}, max_dist: {}", min_dist, max_dist);
        let mut good_matches = core::Vector::<core::DMatch>::new();
        for i in 0..matches.len() {
            if matches.get(i as usize).unwrap().distance < 2.0 * min_dist.max(20.0) {
                good_matches.push(matches.get(i as usize).unwrap());
            }
        }
        println!("good matches size: {}", good_matches.len());
        self.good_matches = good_matches;
        self.keypoint1 = keypoints1;
        self.keypoint2 = keypoints2;
        self.descriptors1 = descriptors1;
        self.descriptors2 = descriptors2;
    }

    // 根据深度图找到对应的3d 和2d点
    fn find_3d_2d_pairs(&mut self) {
        let img_depth = imgcodecs::imread(&self.img_depth, imgcodecs::IMREAD_UNCHANGED).unwrap();
        for m in &self.good_matches {
            // keypoint1 的 x y 像素
            let kp = self.keypoint1.get(m.query_idx as usize).unwrap();
            // img_depth[kp.pt().x as usize][kp.pt().y as usize];
            let depth = img_depth
                .at_2d::<u16>(kp.pt().y as i32, kp.pt().x as i32)
                .unwrap();
            if *depth == 0 {
                continue;
            }
            let d = *depth as f32 / 5000.0;
            let camaera_point = self.pixel_to_camaera(&kp.pt());

            self.object_points.push(core::Point3f::new(
                camaera_point.x * d,
                camaera_point.y * d,
                d,
            ));

            let kp2 = self.keypoint2.get(m.train_idx as usize).unwrap();
            self.image_points.push(self.pixel_to_camaera(&kp2.pt()));
        }
    }

    // pnp 求解
    pub fn solve_pnp(&mut self) {
        self.find_3d_2d_pairs();
        let mut rvec = core::Mat::default();
        let mut tvec = core::Mat::default();
        opencv::calib3d::solve_pnp(
            &self.object_points,
            &self.image_points,
            &self.camera_matrix,
            &core::Mat::default(),
            &mut rvec,
            &mut tvec,
            false,
            opencv::calib3d::SOLVEPNP_EPNP,
        )
        .unwrap();
        // 旋转向量形式，用Rodrigues公式转换为矩阵
        let mut rmat = core::Mat::default();
        opencv::calib3d::rodrigues(&rvec, &mut rmat, &mut core::Mat::default()).unwrap();
        println!("rmat: {:?}", rmat.to_vec_2d::<f64>());
        println!("tvec: {:?}", tvec.to_vec_2d::<f64>());
    }

    // 像素坐标转相机坐标
    fn pixel_to_camaera(&self, p: &core::Point2f) -> core::Point2f {
        let fx = self.camera_matrix.at_2d::<f32>(0, 0).unwrap();
        let fy = self.camera_matrix.at_2d::<f32>(1, 1).unwrap();
        let cx = self.camera_matrix.at_2d::<f32>(0, 2).unwrap();
        let cy = self.camera_matrix.at_2d::<f32>(1, 2).unwrap();
        core::Point2f::new((p.x - cx) / fx, (p.y - cy) / fy)
    }

    pub fn draw_matches(&mut self) {
        // 绘制匹配点
        let mut img1 = imgcodecs::imread(&self.img1, imgcodecs::IMREAD_COLOR).unwrap();
        let mut img2 = imgcodecs::imread(&self.img2, imgcodecs::IMREAD_COLOR).unwrap();
        let mut out_image = core::Mat::default();
        opencv::features2d::draw_matches(
            &mut img1,
            &self.keypoint1,
            &mut img2,
            &self.keypoint2,
            &self.good_matches,
            &mut out_image,
            core::VecN([0., 255., 0., 255.]),
            core::VecN([100., 255., 100., 255.]),
            &core::Vector::<i8>::new(),
            opencv::features2d::DrawMatchesFlags::DEFAULT,
        )
        .unwrap();

        highgui::named_window("orb", highgui::WINDOW_NORMAL).unwrap();
        // 显示图片
        highgui::resize_window("orb", 255, 255).unwrap();
        highgui::imshow("orb", &out_image).unwrap();
        // highgui::imshow("orb", &out_image).unwrap();
        highgui::wait_key(0).unwrap(); // 等待按键退出
        println!("opencv exit");
    }
}

impl PoseEstimation {
    fn detect_keypoint_desc(
        &self,
        img: &mut core::Mat,
        key_point: &mut core::Vector<core::KeyPoint>,
        desc: &mut core::Mat,
    ) {
        let mut orb = <dyn opencv::features2d::ORB>::create(
            500,                                             // 特征点数量
            1.2,                                             // 金字塔缩放比例
            8,                                               // 金字塔层数
            31,                                              // 每个特征点的像素邻域大小
            0,                                               // 特征点阈值
            2,                                               // 金字塔初始层数
            opencv::features2d::ORB_ScoreType::HARRIS_SCORE, // HARRIS_SCORE
            31,                                              // FAST_THRESHOLD
            20,
        )
        .unwrap();
        let mut mask = core::Mat::default();
        orb.detect_and_compute(img, &mut mask, key_point, desc, false)
            .unwrap();
    }

    // 通过BA优化求解
    pub fn ba_slove(&self) {
        use crate::ceres_ba_bind::ffi::*;
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
        let mut pose = Isometry3::new(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::y() * std::f32::consts::FRAC_PI_2,
        );
        let fx = self.camera_matrix.at_2d::<f32>(0, 0).unwrap();
        let fy = self.camera_matrix.at_2d::<f32>(1, 1).unwrap();
        let cx = self.camera_matrix.at_2d::<f32>(0, 2).unwrap();
        let cy = self.camera_matrix.at_2d::<f32>(1, 2).unwrap();
        for iter in 0..100 {
            let mut H = Matrix6::<f32>::zeros();
            let mut b = Vector6::<f32>::zeros();
            // 计算所有的误差 和雅可比
            let mut error = 0.0;
            for i in 0..self.object_points.len() {
                let point3d = self.object_points.get(i).unwrap();
                let point2d = self.image_points.get(i).unwrap();
                // 转化为相机坐标系
                let point_in_caram = pose.inverse() * Vector3::new(point3d.x, point3d.y, point3d.z);
                let inv_z = 1.0 / point_in_caram.z;
                // 像素坐标
                let pixel_point = Vector2::new(
                    fx * point_in_caram.x * inv_z + cx,
                    fy * point_in_caram.y * inv_z + cy,
                );
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
                b += -jacobian.transpose() * cur_e;
            }
            // H.ldlt().solve_mut(&mut b);
            let dx = H.lu().solve(&b).unwrap();
            // let dx = H.try_inverse().unwrap() * b;
            use crate::se3;
            pose = se3::exp(dx) * pose;
            println!("iter: {}, error: {}", iter, error);
            if dx.norm() < 1e-6 {
                break;
            }
        }
        println!("pose: {:?}", pose);
    }
}
