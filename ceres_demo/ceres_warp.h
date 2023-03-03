#ifndef CERES_WAPR_H
#define CERES_WAPR_H
#include <iostream>
#include "ceres/ceres.h"

using namespace std;

class CeresWarp
{
    // 定义损失函数
    struct CURVE_FITTER_COST
    {
        CURVE_FITTER_COST(double x, double y) : _x(x), _y(y) {}
        template <typename T>
        bool operator()(const T *const m, T *residual) const
        {
            // e = y - exp(a*x^2+b*x+c) e是残差，y是观测值，a,b,c是待优化参数 e 是一维的
            residual[0] = T(_y) - ceres::exp(m[0] * T(_x) * T(_x) + m[1] * T(_x) + m[2]);
            return true;
        }
        const double _x, _y;
    };

public:
    void add_residual_block(double x, double y);

    void solve();

private:
    // 定义待优化变量
    double abc[3] = {0.0, 0.0, 0.0};
    ceres::Problem problem;
};

#include <memory>
std::unique_ptr<CeresWarp> new_ceres_warp();
#endif