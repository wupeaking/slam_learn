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
    pub fn solve_carame_pose(&mut self) -> core::Mat {
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
        e_mat
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
}
