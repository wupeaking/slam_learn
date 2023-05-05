#ifndef POSE_H
#define POSE_H
#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "Eigen/Core"
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "rust/cxx.h"

using namespace std;

class POSEOPT
{
    // 定义损失函数
    struct CURVE_FITTER_COST
    {
        // delta_pose[6] = {delta_angle_axis[3], delta_translate[3]}
        CURVE_FITTER_COST(rust::Slice<const double> pose_ij, rust::Slice<const double> info_matrix)

        {
            for (int i = 0; i < 6; i++)
            {
                this->delta_pose[i] = pose_ij[i];
            }
            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    this->info_matrix[i][j] = info_matrix[i * 6 + j];
                }
            }
        }

        template <typename T>
        bool operator()(const T *const angle_aixs_i, const T *const translate_i, const T *const angle_aixs_j, const T *const translate_j, T *residual) const
        { // angle_aixs_i 表示Rwi
            // translate_i 表示twi
            // translate_i - translate_j 表示两帧之间的相对位移 twij 即相对位移在世界坐标系下的表示
            // 将其转为在第i帧坐标系下的表示 Rwi^(-1)*(twi - twj)
            // 或者也可以这里理解 i点的世界坐标

            // 位姿图优化的原理是
            // 1. 通过位置估计到的两帧的位姿Ti, Tj 可以得到相对位姿DeltaTij
            // 2. 通过其他方式如IMU 测量得到的两帧的相对位姿Tij
            // 3. 通过最小化两个相对位姿的差值，来优化位姿图

            // 通过测量得到的两帧的相对偏移
            T t_arr[3] = {T(delta_pose[3]), T(delta_pose[4]), T(delta_pose[5])};
            Eigen::Matrix<T, 3, 1> transform_ij(t_arr);
            // 通过测量得到的两帧的相对旋转 旋转向量表示
            T angle_axis_arr[3] = {T(delta_pose[0]), T(delta_pose[1]), T(delta_pose[2])};
            T q_ij_arr[4];                                          // w, x, y, z
            ceres::AngleAxisToQuaternion(angle_axis_arr, q_ij_arr); // 返回的顺序是 w x y z
            // Eigen::Quaterniond q1(w, x, y, z);// 第一种方式
            // Eigen::Quaterniond q2(Vector4d(x, y, z, w));// 第二种方式
            Eigen::Quaternion<T> q_ij(q_ij_arr[0], q_ij_arr[1], q_ij_arr[2], q_ij_arr[3]);

            // 通过位置估计得到的两帧的相对偏移 pj - pi
            // double ti[3] = {double(translate_i[0]), double(translate_i[1]), double(translate_i[2])};
            //  只能把所有类型转为T类型

            Eigen::Matrix<T, 3, 1> delta_t_w = Eigen::Matrix<T, 3, 1>(translate_j[0], translate_j[1], translate_j[2]) -
                                               Eigen::Matrix<T, 3, 1>(translate_i[0], translate_i[1], translate_i[2]);
            // 转到Ti坐标系下
            // 轴角转四元数
            T q_i_arr[4];
            ceres::AngleAxisToQuaternion(angle_aixs_i, q_i_arr);
            Eigen::Quaternion<T> q_i(q_i_arr[0], q_i_arr[1], q_i_arr[2], q_i_arr[3]);
            T q_j_arr[4];
            ceres::AngleAxisToQuaternion(angle_aixs_j, q_j_arr);
            Eigen::Quaternion<T> q_j(q_j_arr[0], q_j_arr[1], q_j_arr[2], q_j_arr[3]);
            Eigen::Matrix<T, 3, 1> delta_t_ij = q_i.inverse() * delta_t_w;
            // 相对旋转
            Eigen::Quaternion<T> delta_q_ij = q_i.inverse() * q_j;

            // DeltaTij 和 Tij 的差值表示
            //   error = [ p_ab - \hat{p}_ab                 ]
            //           [ 2.0 * Vec(q_ab * \hat{q}_ab^{-1}) ]
            Eigen::Matrix<T, 3, 1> t = delta_t_ij - transform_ij;
            // residual[0] = t[0];
            // residual[1] = t[1];
            // residual[2] = t[2];
            Eigen::Quaternion<T> q_err = delta_q_ij.inverse() * q_ij;
            T delta_angle_axis_arr[3];
            T q_err_arr[4];
            q_err_arr[0] = q_err.w();
            q_err_arr[1] = q_err.x();
            q_err_arr[2] = q_err.y();
            q_err_arr[3] = q_err.z();
            ceres::QuaternionToAngleAxis(q_err_arr, delta_angle_axis_arr);
            Eigen::Matrix<T, 6, 1> delta;
            delta[0] = t[0];
            delta[1] = t[1];
            delta[2] = t[2];
            delta[3] = delta_angle_axis_arr[0];
            delta[4] = delta_angle_axis_arr[1];
            delta[5] = delta_angle_axis_arr[2];

            // residual[3] = delta_angle_axis_arr[0];
            // residual[4] = delta_angle_axis_arr[1];
            // residual[5] = delta_angle_axis_arr[2];

            // 乘上信息矩阵
            Eigen::Matrix<T, 6, 6> info;
            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    info(i, j) = T(info_matrix[i][j]);
                }
            }
            // 对信息矩阵进行LLT分解
            Eigen::LLT<Eigen::Matrix<T, 6, 6>> llt(info);
            // 信息矩阵的上三角矩阵 L^T
            Eigen::Matrix<T, 6, 6> upper = llt.matrixU();
            delta = upper * delta;
            residual[0] = delta[0];
            residual[1] = delta[1];
            residual[2] = delta[2];
            residual[3] = delta[3];
            residual[4] = delta[4];
            residual[5] = delta[5];
            return true;
        }

        double delta_pose[6];     //
        double info_matrix[6][6]; // 信息矩阵 保存已经进行LLT分解的上三角信息矩阵
    };

public:
    POSEOPT(int pose_num) : pose_num(pose_num)
    {
        angleAxis = new double[3 * pose_num];
        t = new double[3 * pose_num];
    };

    ~POSEOPT()
    {
        delete[] angleAxis;
        delete[] t;
    }

    void add_residual_block(rust::Slice<const double> pose_ij, rust::Slice<const double> info_matrix, int pose_i, int pose_j);

    void solve();

    rust::Vec<double> get_pose(int pose_index);
    void set_angle_axis(int pose_index, double x, double y, double z);
    void set_translate(int pose_index, double x, double y, double z);

private:
    // 定义待优化变量 旋转向量
    double *angleAxis;
    double *t;
    int pose_num;
    ceres::Problem problem;
};

#include <memory>
std::unique_ptr<POSEOPT> new_pose_opt(int pose_num);
#endif