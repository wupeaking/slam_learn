/*!
 视觉里程计中的位姿估计主要有三种类型
 1. 2d到2d的位姿估计
    主要是通过特征点的匹配，然后通过对极约束，求出本质矩阵，然后通过本质矩阵求出相机的R t
    然后使用三角测量的方法求出深度信息。需要注意的是单目相机求出的深度信息和实际
    的深度是有一个尺度因子的，所以需要通过其他的方法来求出尺度因子。
 2. 3d到2d的位姿估计
    3d-2d求解相机的R t有直接法,Pnp,BA优化等方法。
    直接法就是定义rt的变量，然后直接进行转换，解出方程组，但是这个时候得到的R不满足旋转矩阵的性质， 一般需要进行QR分解
    BA优化是通过最小化重投影误差来求解相机的R t
 3. 3d到3d的位姿估计
    ICP算法求解 R, t
*/

use nalgebra::{Isometry3, Matrix3, Vector3, Vector6};
use nalgebra::{Matrix2x6, Vector2};
use opencv::core;
use opencv::highgui;
use opencv::imgcodecs;
use opencv::imgproc;
use opencv::prelude::*;

// 3d点到2d点转换 求出R t
pub struct PoseEstimation<T: core::VectorElement, B: core::VectorElement>
where
    core::Vector<T>: core::VectorExtern<T>,
    core::Vector<B>: core::VectorExtern<B>,
{
    // 相机内参
    pub camera_matrix: core::Mat,
    // 3D点
    pub object_points: core::Vector<T>,
    // 2D点
    pub image_points: core::Vector<B>,
    // 匹配的特征点
    pub good_matches: core::Vector<core::DMatch>,

    pub keypoint1: core::Vector<core::KeyPoint>,
    pub keypoint2: core::Vector<core::KeyPoint>,
    pub descriptors1: core::Mat,
    pub descriptors2: core::Mat,
    pub img1: String,
    pub img2: String,
    pub img_depth: String,
}

impl<T: core::VectorElement, B: core::VectorElement> PoseEstimation<T, B>
where
    core::Vector<T>: core::VectorExtern<T>,
    core::Vector<B>: core::VectorExtern<B>,
{
    pub fn new(camera_matrix: core::Mat, img1: String, img2: String, img_depth: String) -> Self {
        let object_points = core::Vector::<T>::new();
        let image_points = core::Vector::<B>::new();
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
            if matches.get(i as usize).unwrap().distance < (2.0 * min_dist).max(20.0) {
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

    // 像素坐标转相机坐标
    pub fn pixel_to_camaera(&self, p: &core::Point2f) -> core::Point2f {
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

impl<T: core::VectorElement, B: core::VectorElement> PoseEstimation<T, B>
where
    core::Vector<T>: core::VectorExtern<T>,
    core::Vector<B>: core::VectorExtern<B>,
{
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

    // 三角测量计算深度
    // p1: 特征点在图像1中的像素坐标
    // p2: 特征点在图像2中的像素坐标
    // camera_matrix: 相机内参
    // r: 旋转矩阵 R21
    // t: 平移矩阵 t21 相机2坐标系下的平移向量
    // 返回值 (s1, s2)  s1: 特征点在图像1中的深度 s2: 特征点在图像2中的深度
    pub fn triangulation(
        &self,
        p1: &core::Point2f,
        p2: &core::Point2f,
        r: &core::Mat,
        t: &core::Mat,
    ) -> (f32, f32) {
        let P1 = self.pixel_to_camaera(p1);
        let P2 = self.pixel_to_camaera(p2);
        let x1 = Vector3::new(P1.x, P1.y, 1.0);
        let x2 = Vector3::new(P2.x, P2.y, 1.0);
        let r_vec = r.to_vec_2d::<f64>().unwrap();
        let r = Matrix3::new(
            r_vec[0][0] as f32,
            r_vec[0][1] as f32,
            r_vec[0][2] as f32,
            r_vec[1][0] as f32,
            r_vec[1][1] as f32,
            r_vec[1][2] as f32,
            r_vec[2][0] as f32,
            r_vec[2][1] as f32,
            r_vec[2][2] as f32,
        );
        let t_vec = t.to_vec_2d::<f64>().unwrap();
        let t = Vector3::new(t_vec[0][0] as f32, t_vec[1][0] as f32, t_vec[2][0] as f32);
        let x2_hat = Matrix3::new(0.0, -x2[2], x2[1], x2[2], 0.0, -x2[0], -x2[1], x2[0], 0.0);
        let tmp1 = x2_hat * r * t;
        let tmp2 = x2_hat * t;
        let s1 = -tmp2.norm() / tmp1.norm();
        // s2x2 = s1Rtx1+t
        let s2 = (r * x1 * s1 + t).norm() / x2.norm();
        (s1, s2)
    }
}
