use nalgebra::Vector2;
use opencv::{self, core, imgcodecs, prelude::*};

// lk光流追踪
pub struct LKFlow {
    pub img1: String,
    pub img2: String,
    img1_gray: core::Mat,
    img2_gray: core::Mat,

    pub keypoint1: core::Vector<core::KeyPoint>,
    pub keypoint2: core::Vector<core::KeyPoint>,
}

impl LKFlow {
    pub fn new(img1: String, img2: String) -> Self {
        Self {
            img1,
            img2,
            img1_gray: core::Mat::default(),
            img2_gray: core::Mat::default(),
            keypoint1: core::Vector::<core::KeyPoint>::new(),
            keypoint2: core::Vector::<core::KeyPoint>::new(),
        }
    }

    // 使用orb特征点检测
    pub fn orb_detect_keypoints(&mut self) {
        let img1 = imgcodecs::imread(&self.img1, imgcodecs::IMREAD_GRAYSCALE).unwrap();
        let img2 = imgcodecs::imread(&self.img2, imgcodecs::IMREAD_GRAYSCALE).unwrap();
        self.img1_gray = img1.clone();
        self.img2_gray = img2.clone();

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
        orb.detect(&img1, &mut self.keypoint1, &mut mask).unwrap();
        orb.detect(&img2, &mut self.keypoint2, &mut mask).unwrap();
    }

    fn get_pixel_value(&self, x: i32, y: i32, img: &core::Mat) -> Option<f64> {
        if x >= 0 && x < self.img1_gray.cols() && y >= 0 && y < self.img1_gray.rows() {
            return Some(*img.at_2d::<u8>(y as i32, x as i32).unwrap() as f64);
        }
        None
    }

    // 通过手写gn优化追踪特征点
    // 求出 dx, dy
    // 误差函数为 e = ||I(x, y) - I(x + dx, y + dy)||^2
    // 目标就是求出dx dy 使得误差函数最小
    // 雅可比J = - I(x + dx, y + dy)关于dx dy的偏导数 dx dy 最小的增量是1个像素
    // J = -[
    //    I(x + dx+1, y + dy) - I(x + dx-1, y + dy)/2,
    //    I(x + dx, y + dy+1) - I(x + dx, y + dy-1)/2
    //  ]
    // J^T * J * delta = -J^T * e 或者 J*J^T * delta = -J * eT   看分子还是分母布局
    pub fn gn_lk_flow(&self, kp: &core::KeyPoint) -> (i32, i32) {
        // 定义初始值
        let mut dx = 0;
        let mut dy = 0;
        let iter = 10;
        let mut last_cost = 0.0;
        // 从特征点的周围取一个n*n的区域
        let n = 4;
        for i in 0..iter {
            // 计算雅可比矩阵
            let mut cost = 0.0;
            let mut H = nalgebra::Matrix2::<f64>::new(0.0, 0.0, 0.0, 0.0);
            let mut b = Vector2::<f64>::new(0.0, 0.0);
            for u in -n..n {
                for v in -n..n {
                    let x = kp.pt().x as i32 + u; // 列坐标   坐标系是左上角 x向右 y向下
                    let y = kp.pt().y as i32 + v; // 行坐标
                                                  // 获取图像灰度值
                    let img1_gray = if let Some(gray) = self.get_pixel_value(y, x, &self.img1_gray)
                    {
                        gray
                    } else {
                        continue;
                    };
                    let img2_gray =
                        if let Some(gray) = self.get_pixel_value(y + dy, x + dx, &self.img2_gray) {
                            gray
                        } else {
                            continue;
                        };

                    let e = (img1_gray - img2_gray) as f64;
                    cost += e * e;

                    let gray_x_add = if let Some(gray) =
                        self.get_pixel_value(y + dy, x + dx + 1, &self.img2_gray)
                    {
                        gray
                    } else {
                        continue;
                    };
                    let gray_x_sub = if let Some(gray) =
                        self.get_pixel_value(y + dy, x + dx - 1, &self.img2_gray)
                    {
                        gray
                    } else {
                        continue;
                    };
                    let gray_x = (gray_x_add - gray_x_sub) / 2.0;

                    let gray_y_add = if let Some(gray) =
                        self.get_pixel_value(y + dy + 1, x + dx, &self.img2_gray)
                    {
                        gray
                    } else {
                        continue;
                    };
                    let gray_y_sub = if let Some(gray) =
                        self.get_pixel_value(y + dy - 1, x + dx, &self.img2_gray)
                    {
                        gray
                    } else {
                        continue;
                    };
                    let gray_y = (gray_y_add - gray_y_sub) / 2.0;

                    let J = Vector2::<f64>::new(gray_x as f64, gray_y as f64) * -1.0;
                    // 计算hessian矩阵
                    H += J * J.transpose();
                    b += -J * e;
                    // println!("b : {}, e: {}", b, e);
                }
            }
            if i > 0 && cost >= last_cost {
                break;
            }
            last_cost = cost;
            // let delta = H.try_inverse().unwrap() * b;
            let delta = if let Some(d) = H.lu().solve(&b) {
                d
            } else {
                break;
            };
            dx += delta[0] as i32;
            dy += delta[1] as i32;
            // println!(
            //     "iter: {},  dx: {}, dy: {}, delta: {} cost: {} ",
            //     i, dx, dy, delta, cost
            // );
            if delta.norm() < 0.001 as f64 {
                break;
            }
        }
        (dx, dy)
    }

    // 追踪光流
    pub fn flow(&mut self) {
        self.orb_detect_keypoints();
        for kp in &self.keypoint1 {
            let (dx, dy) = self.gn_lk_flow(&kp);
            println!("kp: {:?}, dx: {}, dy: {}", kp.pt(), dx, dy);
        }
    }

    pub fn opencv_flow(&mut self) {
        // 使用opencv的光流追踪
        self.orb_detect_keypoints();
        use opencv::types::{VectorOfPoint2f, VectorOff32, VectorOfu8};
        let mut prev_pts = VectorOfPoint2f::new();
        for kp in &self.keypoint1 {
            prev_pts.push(core::Point2f::new(kp.pt().x, kp.pt().y));
        }
        let mut next_pts = VectorOfPoint2f::new();
        let mut status = VectorOfu8::new();
        let mut err = VectorOff32::new();

        opencv::video::calc_optical_flow_pyr_lk(
            &self.img1_gray,
            &self.img2_gray,
            &mut prev_pts,
            &mut next_pts,
            &mut status,
            &mut err,
            core::Size::new(21, 21),
            3,
            core::TermCriteria::new(1 | 2, 30, 0.01).unwrap(), /* C++中有这个定义 enum Type
                                                               {
                                                                   COUNT=1, //!< the maximum number of iterations or elements to compute
                                                                   MAX_ITER=COUNT, //!< ditto
                                                                   EPS=2 //!< the desired accuracy or change in parameters at which the iterative algorithm stops
                                                               };*/
            0,
            0.001,
        )
        .unwrap();

        for i in 0..prev_pts.len() {
            let p1 = prev_pts.get(i).unwrap();
            let p2 = next_pts.get(i).unwrap();
            println!(
                "p1: {:?}, p2: {:?} status: {}, err: {}",
                p1,
                p2,
                status.get(i).unwrap() == 1,
                err.get(i).unwrap()
            );
        }
    }
}

// test

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_gn_lk_flow() {
        let mut lk = LKFlow::new(String::from("data/1.png"), String::from("data/2.png"));
        lk.orb_detect_keypoints();
        let kp = lk.keypoint1.get(0).unwrap();
        let (dx, dy) = lk.gn_lk_flow(&kp);
        println!("kp: {:?},  dx: {}, dy: {}", kp.pt(), dx, dy);
    }
}
