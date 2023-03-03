// 通过BA优化来计算3d到2d的R t

#include "ceres_ba.h"
#include <vector>

std::unique_ptr<CeresBA> new_ceres_ba()
{
    return std::unique_ptr<CeresBA>(new CeresBA());
}

void CeresBA::add_residual_block(double u, double v, double x, double y, double z)
{
    // 向问题中添加误差项
    problem.AddResidualBlock(
        // 使用自动求导，模板参数：误差类型，输出维度，参数块1的维度，参数块2的维度...
        new ceres::AutoDiffCostFunction<CURVE_FITTER_COST, 2, 3, 3>(new CURVE_FITTER_COST(u, v, x, y, z)),
        // 核函数，这里不使用，为空
        nullptr,
        // 待估计参数
        angleAxis, t);
}

void CeresBA::solve()
{
    // 配置求解器
    ceres::Solver::Options options;
    // 打印到cout
    options.minimizer_progress_to_stdout = true;
    // 增量方程如何来计算
    options.linear_solver_type = ceres::DENSE_QR;
    // 增量方程如何来计算
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations = 100;
    // 求解器
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "estimated angleAxis = ";
    for (auto a : angleAxis)
        std::cout << a << " ";

    std::cout << "estimated translate = ";
    for (auto a : t)
        std::cout << a << " ";
    std::cout << std::endl;
}

rust::Vec<double> CeresBA::get_angle_axis()
{
    rust::Vec<double> slice;
    slice.push_back(angleAxis[0]);
    slice.push_back(angleAxis[1]);
    slice.push_back(angleAxis[2]);
    return slice;
}

rust::Vec<double> CeresBA::get_translate()
{
    rust::Vec<double> slice;
    slice.push_back(t[0]);
    slice.push_back(t[1]);
    slice.push_back(t[2]);
    return slice;
}