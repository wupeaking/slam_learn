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
use nalgebra::{Isometry3, Matrix6, Vector3, Vector6};
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
}
