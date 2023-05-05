use anyhow::{self, Result};
/// 后端位姿图的优化
/// 位姿图优化的主要工作是对位姿图中的节点和边进行优化，使得优化后的位姿图能够更好地反映相机的运动轨迹。
/// 由于BA优化的过程中路标点过大，因此我们在优化的过程中只优化相机的位姿，而不优化路标点的位置。
/// 在相机的运动过程中， 可以评估到每次相机的位姿。 同时也可能通过另外的传感器得到两个相机之间的相对位姿。
/// 通过这些信息，我们可以构建一个位姿图，然后通过优化位姿图来得到更加准确的相机位姿。
/// 本示例中使用g2o生成的示例数据，来进行优化。 使用ceres进行优化。
/// ceres针对该示例有个优化的示例代码: https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/slam/pose_graph_3d/pose_graph_3d_error_term.h
///
/// ceres的示例代码优化的是四元数 本示例优化的是轴角 因为四元数是4个自由度表示三个自由度的旋转 所以有一定的约束 需要设置LocalParameterization
/// LocalParameterization是在优化Manifold(流形)上的变量时需要考虑的，Manifold上变量是过参数的，即Manifold上变量的维度大于其自由度。
/// 这会导致Manifold上变量各个量之间存在约束，如果直接对这些量求导、优化，那么这就是一个有约束的优化，实现困难。为了解决这个问题，在数学上对Manifold在当前变量值处形成的切空间求导，在切空间上优化，最后投影回Manifold。
/// 示例文件的格式:
//
// VERTEX_SE3:QUAT ID x y z q_x q_y q_z q_w
//
// where the quaternion is in Hamilton form.
// A constraint is defined as follows:
// 信息矩阵的上三角元素 信息矩阵是协方差矩阵的逆
// EDGE_SE3:QUAT ID_a ID_b x_ab y_ab z_ab q_x_ab q_y_ab q_z_ab q_w_ab I_11 I_12 I_13 ... I_16 I_22 I_23 ... I_26 ... I_66 // NOLINT
//
// where I_ij is the (i, j)-th entry of the information matrix for the
// measurement. Only the upper-triangular part is stored. The measurement order
// is the delta position followed by the delta orientation.

/// 优化之后的图形展示
/// python3 polt.py --initial_poses poses.txt --optimized_poses poses_opt.txt
use nalgebra::{Matrix3, Matrix6, Unit, UnitQuaternion, Vector3};
use std::collections::HashMap;
use std::fs::File;
use std::io::Write;

struct Pose {
    // 位姿 旋转和平移 旋转使用四元数
    rotation: UnitQuaternion<f64>,
    // 转为轴角
    axis_angle: Vector3<f64>,
    translation: Vector3<f64>,
    index: i32,
}

// 位姿约束
struct PoseConstraint {
    // 位姿约束
    // 两个位姿之间的相对位姿
    // 旋转使用四元数
    rotation: UnitQuaternion<f64>,
    // 转为轴角
    axis_angle: Vector3<f64>,
    translation: Vector3<f64>,
    // 信息矩阵
    information: Matrix6<f64>,
    // 两个位姿的索引
    index0: i32,
    index1: i32,
}
pub struct PoseOpt {
    poses: HashMap<i32, Pose>,
    constraints: Vec<PoseConstraint>,
    opt_poses: Vec<Pose>,
}

impl PoseOpt {
    pub fn new(file: &str) -> Result<Self> {
        use std::fs::File;
        use std::io::{BufRead, BufReader};
        let file = File::open(file)?;
        let reader = BufReader::new(file);
        let mut opt = Self {
            poses: HashMap::new(),
            constraints: Vec::new(),
            opt_poses: Vec::new(),
        };
        for (i, line) in reader.lines().enumerate() {
            let line = line?;
            let mut iter = line.split_whitespace();
            let t: String = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
            match t.as_str() {
                "VERTEX_SE3:QUAT" => {
                    let index: i32 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                    let x: f64 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                    let y: f64 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                    let z: f64 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                    let q_x: f64 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                    let q_y: f64 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                    let q_z: f64 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                    let q_w: f64 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                    let rotation = UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
                        q_w, q_x, q_y, q_z,
                    ));
                    // 四元数转为轴角
                    let axis_angle = rotation.axis_angle();
                    let axis_angle = if axis_angle.is_none() {
                        Vector3::new(0.0, 0.0, 0.0)
                    } else {
                        Vector3::new(
                            axis_angle.unwrap().0.x,
                            axis_angle.unwrap().0.y,
                            axis_angle.unwrap().0.z,
                        ) * axis_angle.unwrap().1
                    };
                    let translation = Vector3::new(x, y, z);
                    opt.poses.insert(
                        index,
                        Pose {
                            rotation,
                            axis_angle,
                            translation,
                            index,
                        },
                    );
                }
                "EDGE_SE3:QUAT" => {
                    let index0: i32 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                    let index1: i32 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                    let x: f64 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                    let y: f64 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                    let z: f64 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                    let q_x: f64 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                    let q_y: f64 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                    let q_z: f64 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                    let q_w: f64 = iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                    let rotation = UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
                        q_w, q_x, q_y, q_z,
                    ));
                    // 四元数转为轴角
                    let axis_angle = rotation.axis_angle();
                    let axis_angle = if axis_angle.is_none() {
                        Vector3::new(0.0, 0.0, 0.0)
                    } else {
                        Vector3::new(
                            axis_angle.unwrap().0.x,
                            axis_angle.unwrap().0.y,
                            axis_angle.unwrap().0.z,
                        ) * axis_angle.unwrap().1
                    };

                    let translation = Vector3::new(x, y, z);
                    let mut information = Matrix6::zeros();
                    for i in 0..6 {
                        for j in i..6 {
                            let v: f64 =
                                iter.next().ok_or(anyhow::anyhow!("parse err"))?.parse()?;
                            information[(i, j)] = v;
                            if i != j {
                                information[(j, i)] = v;
                            }
                        }
                    }
                    opt.constraints.push(PoseConstraint {
                        rotation,
                        axis_angle,
                        translation,
                        information,
                        index0,
                        index1,
                    });
                }
                _ => {
                    return Err(anyhow::anyhow!("parse err"));
                }
            }
        }

        Ok(opt)
    }

    // 优化
    pub fn optimization(&mut self) {
        use super::pose_bind::ffi::*;
        unsafe {
            let mut pose_opt = new_pose_opt(self.poses.len() as i32);
            // 设置位姿的初始值
            for pose in self.poses.values() {
                pose_opt.as_mut().unwrap().set_angle_axis(
                    pose.index,
                    pose.axis_angle.x,
                    pose.axis_angle.y,
                    pose.axis_angle.z,
                );
                pose_opt.as_mut().unwrap().set_translate(
                    pose.index,
                    pose.translation.x,
                    pose.translation.y,
                    pose.translation.z,
                );
            }
            // 设置位姿的约束
            for constraint in self.constraints.iter() {
                let pose_ij = [
                    constraint.axis_angle.x,
                    constraint.axis_angle.y,
                    constraint.axis_angle.z,
                    constraint.translation.x,
                    constraint.translation.y,
                    constraint.translation.z,
                ];
                let info_martix = constraint.information.as_slice();
                // println!("info_martix: {:?}", info_martix);
                pose_opt.as_mut().unwrap().add_residual_block(
                    &pose_ij,
                    info_martix,
                    constraint.index0,
                    constraint.index1,
                );
            }

            // 优化
            pose_opt.as_mut().unwrap().solve();

            // 获取优化后的位姿
            for i in 0..self.poses.len() {
                let p = pose_opt.as_mut().unwrap().get_pose(i as i32);
                // 轴角转四元数
                let angle_axis = Vector3::new(p[0], p[1], p[2]);
                let axis = Unit::new_normalize(angle_axis);
                let rotation = UnitQuaternion::from_axis_angle(&axis, angle_axis.norm());
                let translation = Vector3::new(p[3], p[4], p[5]);
                self.opt_poses.push(Pose {
                    rotation,
                    axis_angle: angle_axis,
                    translation,
                    index: i as i32,
                })
            }
        }
    }

    // 保存原始的位姿
    pub fn save_ori_poses(&self, path: &str) -> Result<()> {
        let mut file = File::create(path)?;
        let mut v = self.poses.values().collect::<Vec<_>>();
        v.sort_by(|a, b| a.index.cmp(&b.index));
        for pose in v {
            let q = pose.rotation.quaternion();
            let line = format!(
                "{} {} {} {} {} {} {} {}\n",
                pose.index,
                pose.translation.x,
                pose.translation.y,
                pose.translation.z,
                q.i,
                q.j,
                q.k,
                q.w
            );
            file.write_all(line.as_bytes())?;
        }
        Ok(())
    }

    // 保存优化后的位姿
    pub fn save_opt_poses(&self, path: &str) -> Result<()> {
        let mut file = File::create(path)?;
        for pose in self.opt_poses.iter() {
            let q = pose.rotation.quaternion();
            let line = format!(
                "{} {} {} {} {} {} {} {}\n",
                pose.index,
                pose.translation.x,
                pose.translation.y,
                pose.translation.z,
                q.i,
                q.j,
                q.k,
                q.w
            );
            file.write_all(line.as_bytes())?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_optimization() {
        let mut opt = PoseOpt::new("data/shape.g2o").unwrap();
        opt.optimization();
        opt.save_opt_poses("data/opt_poses.txt").unwrap();
        opt.save_ori_poses("data/ori_poses.txt").unwrap();
    }
}
