// bal数据集格式
/*
<num_cameras> <num_points> <num_observations>
<camera_index_1> <point_index_1> <x_1> <y_1>
...
<camera_index_num_observations> <point_index_num_observations> <x_num_observations> <y_num_observations>
<camera_1>
每个相机9个参数 3个旋转向量 3个平移向量 焦距f k1 k2
<camera_num_cameras>
<point_1>
...
<point_num_points>
*/
/*
目的: 根据已知的观测值 (camera_index, point_index, x, y) 求解相机的内外参
*/
use anyhow::{self, Result};
use nalgebra::{Vector2, Vector3};

// 定义观测值
struct Observation {
    camera_index: usize,
    point_index: usize,
    observation: Vector2<f64>,
}

#[derive(Debug, Clone, Copy)]
struct CameraArg {
    rotation: Vector3<f64>,
    translation: Vector3<f64>,
    focal: f64,
    k1: f64,
    k2: f64,
}

pub struct BALOpt {
    observations: Vec<Observation>,
    points: Vec<Vector3<f64>>,
    camera_args: Vec<CameraArg>,
    camera_opt: Vec<CameraArg>,
    point_opt: Vec<Vector3<f64>>,
}

impl BALOpt {
    pub fn new(file: &str) -> Result<Self> {
        // 按行读取文件
        use std::fs::File;
        use std::io::{BufRead, BufReader};
        let file = File::open(file)?;
        let reader = BufReader::new(file);
        let mut opt = Self {
            observations: Vec::new(),
            points: Vec::new(),
            camera_args: Vec::new(),
            camera_opt: Vec::new(),
            point_opt: Vec::new(),
        };

        let mut cur_camera_args_index = 0_usize;
        let mut cur_camera_args = CameraArg {
            rotation: Vector3::new(0.0, 0.0, 0.0),
            translation: Vector3::new(0.0, 0.0, 0.0),
            focal: 0.0,
            k1: 0.0,
            k2: 0.0,
        };
        let mut cur_point_arg_index = 0_usize;
        let mut cur_point = Vector3::new(0.0, 0.0, 0.0);
        let mut num_cameras = 0_usize;
        let mut num_points = 0_usize;
        let mut num_observations = 0_usize;
        for (i, line) in reader.lines().enumerate() {
            let line = line?;
            if i == 0 {
                // 第一行
                let mut iter = line.split_whitespace();
                num_cameras = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                num_points = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                num_observations = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                println!(
                    "num_cameras: {}, num_points: {}, num_observations: {}",
                    num_cameras, num_points, num_observations
                );
            }
            if i > 0 && i <= num_observations {
                // 观测值
                let mut iter = line.split_whitespace();
                let camera_index: usize =
                    iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                let point_index: usize =
                    iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                let x: f64 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                let y: f64 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                println!(
                    "camera_index: {}, point_index: {}, x: {}, y: {}",
                    camera_index, point_index, x, y
                );
                opt.observations.push(Observation {
                    camera_index,
                    point_index,
                    observation: Vector2::new(x, y),
                });
            }
            // 相机参数 每个相机9个参数
            if i > num_observations && i <= num_observations + num_cameras * 9 {
                let v: f64 = line.parse()?;
                match cur_camera_args_index {
                    0 => cur_camera_args.rotation.x = v,
                    1 => cur_camera_args.rotation.y = v,
                    2 => cur_camera_args.rotation.z = v,
                    3 => cur_camera_args.translation.x = v,
                    4 => cur_camera_args.translation.y = v,
                    5 => cur_camera_args.translation.z = v,
                    6 => cur_camera_args.focal = v,
                    7 => cur_camera_args.k1 = v,
                    8 => cur_camera_args.k2 = v,
                    _ => {}
                }
                cur_camera_args_index += 1;
                if cur_camera_args_index == 9 {
                    // cur_camera_index += 1;
                    cur_camera_args_index = 0;
                    opt.camera_args.push(cur_camera_args);
                }
            }
            // 3d点
            if i > num_observations + num_cameras * 9
                && i <= num_observations + num_cameras * 9 + num_points * 3
            {
                let v: f64 = line.parse()?;
                match cur_point_arg_index {
                    0 => cur_point.x = v,
                    1 => cur_point.y = v,
                    2 => cur_point.z = v,
                    _ => {}
                }
                cur_point_arg_index += 1;
                if cur_point_arg_index == 3 {
                    cur_point_arg_index = 0;
                    opt.points.push(cur_point);
                }
            }
        }
        // 校验
        if opt.points.len() != num_points {
            return Err(anyhow::anyhow!("points len err"));
        }
        if opt.camera_args.len() != num_cameras {
            return Err(anyhow::anyhow!("camera_args len err"));
        }
        if opt.observations.len() != num_observations {
            return Err(anyhow::anyhow!("observations len err"));
        }
        Ok(opt)
    }

    pub fn optimization(&mut self) {
        use super::bal_bind::ffi::*;
        unsafe {
            let mut bal = new_bal(self.camera_args.len() as i32, self.points.len() as i32);
            // 设置待优化参数的初始值
            for camera_index in 0..self.camera_args.len() as i32 {
                let camera_arg = self.camera_args[camera_index as usize];
                let rotation = camera_arg.rotation;
                let translation = camera_arg.translation;
                let focal = camera_arg.focal;
                let k1 = camera_arg.k1;
                let k2 = camera_arg.k2;
                bal.as_mut().unwrap().set_angle_axis(
                    camera_index,
                    rotation.x,
                    rotation.y,
                    rotation.z,
                );
                bal.as_mut().unwrap().set_translate(
                    camera_index,
                    translation.x,
                    translation.y,
                    translation.z,
                );
                bal.as_mut()
                    .unwrap()
                    .set_camera_inter(camera_index, focal, k1, k2);
            }
            for point_index in 0..self.points.len() as i32 {
                let point = self.points[point_index as usize];
                bal.as_mut()
                    .unwrap()
                    .set_landmark(point_index, point.x, point.y, point.z);
            }
            // 添加残差
            for ob in self.observations.iter() {
                let camera_index = ob.camera_index as i32;
                let point_index = ob.point_index as i32;
                let u = ob.observation.x;
                let v = ob.observation.y;
                // let x = self.points[point_index as usize].x;
                // let y = self.points[point_index as usize].y;
                // let z = self.points[point_index as usize].z;
                bal.as_mut()
                    .unwrap()
                    .add_residual_block(u, v, camera_index, point_index);
            }
            // 优化
            bal.as_mut().unwrap().solve();
            // save
            for i in 0..self.camera_args.len() {
                let camera_arg = self.camera_args[i];
                let opt = bal.as_mut().unwrap().get_camera_args(i as i32);
                self.camera_opt.push(CameraArg {
                    rotation: Vector3::new(opt[0], opt[1], opt[2]),
                    translation: Vector3::new(opt[3], opt[4], opt[5]),
                    focal: opt[6],
                    k1: opt[7],
                    k2: opt[8],
                });
            }
            for i in 0..self.points.len() {
                let opt = bal.as_ref().unwrap().get_landmark(i as i32);
                self.point_opt.push(Vector3::new(opt[0], opt[1], opt[2]));
            }
        }
    }

    pub fn print_opt_result(&self, landmark_print: bool) {
        // 打印真实结果
        for (i, (camera_arg, camera_opt)) in self
            .camera_args
            .iter()
            .zip(self.camera_opt.iter())
            .enumerate()
        {
            println!(
                "camera index: {}:
                    rotation:  {:?}, translation: {:?}, focal: {}, k1: {}, k2: {}
                opt: rotation: {:?}, translation: {:?}, focal: {}, k1: {}, k2: {}",
                i,
                camera_arg.rotation,
                camera_arg.translation,
                camera_arg.focal,
                camera_arg.k1,
                camera_arg.k2,
                camera_opt.rotation,
                camera_opt.translation,
                camera_opt.focal,
                camera_opt.k1,
                camera_opt.k2,
            );
        }
        if !landmark_print {
            return;
        }
        for (i, (point, point_opt)) in self.points.iter().zip(self.point_opt.iter()).enumerate() {
            println!(
                "point index: {}:,
                    point: {:?}
                    opt: {:?}",
                i, point, point_opt,
            );
        }
    }
}

// test
#[cfg(test)]
mod tests {
    use super::*;
    //  cargo test test_bal_opt  -- --nocapture
    #[test]
    fn test_bal_opt() {
        let mut opt: BALOpt = BALOpt::new("data/problem.txt").unwrap();
        println!(
            "ob len: {}, camare num: {}, point num: {} ",
            opt.observations.len(),
            opt.camera_args.len(),
            opt.points.len()
        );
        opt.optimization();
        opt.print_opt_result(false);
    }
}
