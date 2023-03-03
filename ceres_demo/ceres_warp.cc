#include "ceres_warp.h"

std::unique_ptr<CeresWarp> new_ceres_warp()
{
    return std::unique_ptr<CeresWarp>(new CeresWarp());
}

void CeresWarp::add_residual_block(double x, double y)
{
    // 向问题中添加误差项
    problem.AddResidualBlock(
        // 使用自动求导，模板参数：误差类型，输出维度，输入维度
        new ceres::AutoDiffCostFunction<CURVE_FITTER_COST, 1, 3>(new CURVE_FITTER_COST(x, y)),
        // 核函数，这里不使用，为空
        nullptr,
        // 待估计参数
        abc);
}

void CeresWarp::solve()
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
    std::cout << "estimated a,b,c = ";
    for (auto a : abc)
        std::cout << a << " ";
    std::cout << std::endl;
}