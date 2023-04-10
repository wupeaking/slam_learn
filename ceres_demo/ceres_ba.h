#ifndef CERES_BA_H
#define CERES_BA_H
#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "rust/cxx.h"

using namespace std;

class CeresBA
{
    // 定义损失函数
    struct CURVE_FITTER_COST
    {
        CURVE_FITTER_COST(double _u, double _v, double _x, double _y, double _z) : x(_x), y(_y), u(_u), v(_v), z(_z) {}
        template <typename T>
        bool operator()(const T *const angle_aixs, const T *const translate, T *residual) const
        { // angle_aixs 旋转向量(轴角) 使用三个变量来表示旋转 之前没有约束条件
            T rotate_point[3];
            T point[3] = {T(x), T(y), T(z)};  // 全局坐标
            // 注意 这里求的是Rcw tcw（相机坐标系下看到全局原点的坐标） 
            // Pc = Rcw*Pw+tcw  
            // ceres::AngleAxisRotatePoint(angle_aixs, point, rotate_point);
            // rotate_point[0] += translate[0];
            // rotate_point[1] += translate[1];
            // rotate_point[2] += translate[2];  

            // 如果angle_aixs 代表相机坐标相对全局坐标的旋转Rwc translate代表相机在全局坐标下的位置（twc）
            // 那么Pw=Rwc*Pc+t_wc   Pc=Rwc'*(Pw-twc)  
            point[0] -= translate[0];
            point[1] -= translate[1];
            point[2] -= translate[2];
            T angle_aixs_invert[3];
            angle_aixs_invert[0] = -angle_aixs[0];
            angle_aixs_invert[1] = -angle_aixs[1];
            angle_aixs_invert[2] = -angle_aixs[2];
            ceres::AngleAxisRotatePoint(angle_aixs_invert, point, rotate_point);

            residual[0] = T(u) - (T(fx) * (rotate_point[0] / rotate_point[2]) + T(cx));
            residual[1] = T(v) - (T(fy) * (rotate_point[1] / rotate_point[2]) + T(cy));
            return true;
        } 
        // http : // ceres-solver.org/nnls_tutorial.html#bundle-adjustment
        // https://www.jianshu.com/p/34cb21e00264

        // 直接定义这样的残差会出错 因为旋转矩阵9个量之间是有约束的 如果写成这样 会导致约束丢失 其实是R不能直接进行想加导致的 现在
        // 使用ceres::AngleAxisRotatePoint来进行旋转  这样要优化的变量就变成了旋转向量(轴角)和平移向量
        // bool operator()(const T *const m, T *residual) const
        // {
        //     /*
        //     e0 = u - (fx * (m[0] * x + m[1] * y + m[2] * z + m[3]) / (m[8] * x + m[9] * y + m[10] * z + m[11])  + cx);
        //     e1 = v - (fx * (m[4] * x + m[5] * y + m[6] * z + m[7]) / (m[8] * x + m[9] * y + m[10] * z + m[11])  + cy);
        //     */
        //     residual[0] = T(u) - (T(fx) * (m[0] * T(x) + m[1] * T(y) + m[2] * T(z) + m[3]) / (m[8] * T(x) + m[9] * T(y) + m[10] * T(z) + m[11]) + T(cx));
        //     residual[1] = T(v) - (T(fy) * (m[4] * T(x) + m[5] * T(y) + m[6] * T(z) + m[7]) / (m[8] * T(x) + m[9] * T(y) + m[10] * T(z) + m[11]) + T(cy));
        //     return true;
        // }
        const double u,
            v,
            x,
            y,
            z;
        // 520.9_f32, 0.0, 325.1, 0.0, 521.0, 249.7, 0.0, 0.0, 1.0
        double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    };

public:
    void add_residual_block(double u, double v, double x, double y, double z);

    void solve();

    rust::Vec<double> get_angle_axis();
    rust::Vec<double> get_translate();

private:
    // 定义待优化变量 旋转向量
    double angleAxis[3] = {};
    double t[3] = {};
    ceres::Problem problem;
};

#include <memory>
std::unique_ptr<CeresBA> new_ceres_ba();
#endif