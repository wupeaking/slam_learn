use super::pose_estimation::PoseEstimation;
use opencv::{core, prelude::*};

impl PoseEstimation<core::Point2f, core::Point2f> {
    fn find_2d_2d_pairs(&mut self) {
        for m in &self.good_matches {
            // keypoint1 的 x y 像素
            let kp = self.keypoint1.get(m.query_idx as usize).unwrap();
            // let camaera_point = self.pixel_to_camaera(&kp.pt());

            self.object_points.push(kp.pt());

            let kp2 = self.keypoint2.get(m.train_idx as usize).unwrap();
            // self.pixel_to_camaera(&kp2.pt())
            self.image_points.push(kp2.pt());

            // println!("kp: {:?} kp2: {:?}", kp.pt(), kp2.pt());
        }
    }
    pub fn solve_carame_pose(&mut self) -> (core::Mat, core::Mat, core::Mat) {
        self.find_2d_2d_pairs();
        let mut rmat = core::Mat::default();
        let mut tvec = core::Mat::default();
        let e_mat = opencv::calib3d::find_essential_mat(
            &self.object_points,
            &self.image_points,
            &self.camera_matrix,
            opencv::calib3d::RANSAC,
            0.999,
            1.0,
            1000,
            &mut core::Mat::default(),
        )
        .unwrap();
        println!("e_mat: {:?}", e_mat.to_vec_2d::<f64>());
        // 获取焦距f
        let f = self.camera_matrix.at_2d::<f32>(0, 0).unwrap();
        // 相机的光心
        let pp = core::Point2d::new(
            self.camera_matrix.at_2d::<f32>(0, 2).unwrap().clone() as f64,
            self.camera_matrix.at_2d::<f32>(1, 2).unwrap().clone() as f64,
        );
        let points_num = opencv::calib3d::recover_pose(
            &e_mat,
            &self.object_points,
            &self.image_points,
            &mut rmat,
            &mut tvec,
            *f as f64,
            pp,
            &mut core::Mat::default(),
        )
        .unwrap();
        println!("rmat: {:?}", rmat.to_vec_2d::<f64>());
        println!("tvec: {:?}", tvec.to_vec_2d::<f64>());
        println!("points num: {:?}", points_num);
        (e_mat, rmat, tvec)
    }

    // 校验对极约束
    pub fn check_epipolar_constraint(&self, e_mat: &core::Mat) {
        for i in 0..self.object_points.len() {
            use nalgebra::{Matrix3, Vector3};
            let p1 = self.object_points.get(i).unwrap();
            let p2 = self.image_points.get(i).unwrap();
            let p1 = Vector3::new(p1.x, p1.y, 1.0);
            let p2 = Vector3::new(p2.x, p2.y, 1.0);
            let e_mat = e_mat.to_vec_2d::<f64>().unwrap();
            let e_mat = Matrix3::new(
                e_mat[0][0] as f32,
                e_mat[0][1] as f32,
                e_mat[0][2] as f32,
                e_mat[1][0] as f32,
                e_mat[1][1] as f32,
                e_mat[1][2] as f32,
                e_mat[2][0] as f32,
                e_mat[2][1] as f32,
                e_mat[2][2] as f32,
            );
            let camera_matrix = self.camera_matrix.to_vec_2d::<f32>().unwrap();
            let camera_matrix = Matrix3::new(
                camera_matrix[0][0],
                camera_matrix[0][1],
                camera_matrix[0][2],
                camera_matrix[1][0],
                camera_matrix[1][1],
                camera_matrix[1][2],
                camera_matrix[2][0],
                camera_matrix[2][1],
                camera_matrix[2][2],
            );
            let camera_matrix_inv = camera_matrix.try_inverse().unwrap();
            let x2 = camera_matrix_inv * p2;
            let x1 = camera_matrix_inv * p1;
            let epipolar_constraint = x2.transpose() * e_mat * x1;
            println!("epipolar_constraint: {:?}", epipolar_constraint);
        }
    }

    // 求解每个匹配点的3d坐标
    pub fn triangulation_match_points(&self, r: &core::Mat, t: &core::Mat) {
        for i in 0..self.object_points.len() {
            let p1 = self.object_points.get(i).unwrap();
            let p2 = self.image_points.get(i).unwrap();
            let (s1, s2) = self.triangulation(&p1, &p2, r, t);
            let p1_in_camera = self.pixel_to_camaera(&p1);
            let p2_in_camera = self.pixel_to_camaera(&p2);
            println!("像素坐标值: {:?}, 相机坐标系下坐标: {:?}, 像素坐标值: {:?}, 相机坐标系下坐标: {:?},", 
            p1, (s1*p1_in_camera.x, s1*p1_in_camera.y, s1), p2, (s2*p2_in_camera.x, s2*p2_in_camera.y, s2));
        }
    }
}

// 添加测试代码
#[cfg(test)]
mod test {
    use super::*;
    use opencv::{core, prelude::*, types::VectorOfMat};

    #[test]
    fn test_solve_carame_pose() {
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
        let mut pose_estimation = PoseEstimation::<core::Point2f, core::Point2f>::new(
            kk,
            "data/1.png".to_string(),
            "data/2.png".to_string(),
            "data/1_depth.png".to_string(),
        );
        // 查找匹配点
        pose_estimation.find_match_keypoints();
        let (e_mat, _, _) = pose_estimation.solve_carame_pose();
        // 打印对极约束
        pose_estimation.check_epipolar_constraint(&e_mat);
    }
}
