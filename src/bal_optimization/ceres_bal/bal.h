#ifndef BAL_H
#define BAL_H
#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "rust/cxx.h"

using namespace std;

class BAL
{
    // 定义损失函数
    struct CURVE_FITTER_COST
    {
        CURVE_FITTER_COST(double _u, double _v) : u(_u), v(_v) {}
        template <typename T>
        bool operator()(const T *const angle_aixs, const T *const translate, const T *const camera, const T *const landmark, T *residual) const
        {
            // angle_aixs 旋转向量(轴角) 使用三个变量来表示旋转 之前没有约束条件
            T rotate_point[3];                                             // 相机坐标系下坐标
            T point[3] = {T(landmark[0]), T(landmark[1]), T(landmark[2])}; // 全局坐标
            // 注意 这里求的是Rcw tcw（相机坐标系下看到全局原点的坐标）
            // Pc = Rcw*Pw+tcw
            ceres::AngleAxisRotatePoint(angle_aixs, point, rotate_point);
            rotate_point[0] += translate[0];
            rotate_point[1] += translate[1];
            rotate_point[2] += translate[2];
            // 归一化坐标 这个BAL数据集中的特别点在于相机坐标系有负Z轴
            T xp = -rotate_point[0] / rotate_point[2];
            T yp = -rotate_point[1] / rotate_point[2];
            // 畸变模型
            T r2 = xp * xp + yp * yp;
            T distortion = T(1.0) + camera[1] * r2 + camera[2] * r2 * r2;
            // 像素坐标
            T fx = camera[0];
            T fy = camera[0];
            T u_predicted = fx * distortion * xp;
            T v_predicted = fy * distortion * yp;

            residual[0] = T(u) - u_predicted;
            residual[1] = T(v) - v_predicted;
            return true;
        }
        const double u,
            v;
        // x,
        // y,
        // z;
        // 520.9_f32, 0.0, 325.1, 0.0, 521.0, 249.7, 0.0, 0.0, 1.0
        // double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    };

public:
    BAL(int camera_num, int landmark_num) : camera_num(camera_num), landmark_num(landmark_num)
    {
        angleAxis = new double[3 * camera_num];
        t = new double[3 * camera_num];
        camera = new double[3 * camera_num];
        landmark = new double[3 * landmark_num];
    };

    ~BAL()
    {
        delete[] angleAxis;
        delete[] t;
        delete[] camera;
        delete[] landmark;
    }

    void add_residual_block(double u, double v, int camera_index, int landmark_index);

    void solve();

    rust::Vec<double> get_angle_axis(int camera_index);
    rust::Vec<double> get_translate(int camera_index);
    rust::Vec<double> get_camera_inter(int camera_index);
    // 相机所有参数
    rust::Vec<double> get_camera_args(int camera_index);
    rust::Vec<double> get_landmark(int landmark_index) const;

    // 设置初始值
    void set_angle_axis(int camera_index, double x, double y, double z);
    void set_translate(int camera_index, double x, double y, double z);
    void set_camera_inter(int camera_index, double f, double k1, double k2);
    void set_landmark(int landmark_index, double x, double y, double z);

private:
    // 定义待优化变量 旋转向量
    double *angleAxis;
    double *t;
    // f k1, k2
    double *camera;
    // 路标点
    double *landmark;

    int camera_num;
    int landmark_num;
    ceres::Problem problem;
};

#include <memory>
std::unique_ptr<BAL> new_bal(int camera_num, int landmark_num);
#endif