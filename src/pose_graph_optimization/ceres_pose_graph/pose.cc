// 通过BA优化来计算3d到2d的R t

#include "pose.h"
#include <vector>

std::unique_ptr<POSEOPT> new_pose_opt(int pose_num)
{
    return std::unique_ptr<POSEOPT>(new POSEOPT(pose_num));
}

void POSEOPT::add_residual_block(rust::Slice<const double> pose_ij, rust::Slice<const double> info_matrix, int pose_i, int pose_j)
{
    // 使用自动求导，模板参数：误差类型，输出维度，参数块1的维度，参数块2的维度...
    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CURVE_FITTER_COST, 6, 3, 3, 3, 3>(new CURVE_FITTER_COST(pose_ij, info_matrix));
    // 向问题中添加误差项
    problem.AddResidualBlock(
        cost_function,
        // 核函数，这里不使用，为空
        nullptr,
        // new ceres::HuberLoss(1.0),
        // 待估计参数
        angleAxis + 3 * pose_i,
        t + 3 * pose_i,
        angleAxis + 3 * pose_j,
        t + 3 * pose_j);
}

void POSEOPT::solve()
{
    // 配置求解器
    ceres::Solver::Options options;
    // 打印到cout
    options.minimizer_progress_to_stdout = true;
    // 增量方程如何来计算
    options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
    // 增量方程如何来计算
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations = 100;
    // 求解器
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    std::cout << std::endl;
}

rust::Vec<double> POSEOPT::get_pose(int pose_index)
{
    rust::Vec<double> slice;
    auto ptr = angleAxis + 3 * pose_index;
    slice.push_back(ptr[0]);
    slice.push_back(ptr[1]);
    slice.push_back(ptr[2]);
    auto t_ptr = t + 3 * pose_index;
    slice.push_back(t_ptr[0]);
    slice.push_back(t_ptr[1]);
    slice.push_back(t_ptr[2]);
    return slice;
}

void POSEOPT::set_angle_axis(int pose_index, double x, double y, double z)
{
    auto ptr = angleAxis + 3 * pose_index;
    ptr[0] = x;
    ptr[1] = y;
    ptr[2] = z;
}

void POSEOPT::set_translate(int pose_index, double x, double y, double z)
{
    auto ptr = t + 3 * pose_index;
    ptr[0] = x;
    ptr[1] = y;
    ptr[2] = z;
}