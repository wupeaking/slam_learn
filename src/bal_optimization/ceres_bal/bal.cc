// 通过BA优化来计算3d到2d的R t

#include "bal.h"
#include <vector>

std::unique_ptr<BAL> new_bal(int camera_num, int landmark_num)
{
    return std::unique_ptr<BAL>(new BAL(camera_num, landmark_num));
}

void BAL::add_residual_block(double u, double v, int camera_index, int landmark_index)
{
    // 使用自动求导，模板参数：误差类型，输出维度，参数块1的维度，参数块2的维度...
    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CURVE_FITTER_COST, 2, 3, 3, 3, 3>(new CURVE_FITTER_COST(u, v));
    // 向问题中添加误差项
    problem.AddResidualBlock(
        cost_function,
        // 核函数，这里不使用，为空
        nullptr,
        // new ceres::HuberLoss(1.0),
        // 待估计参数
        angleAxis + 3 * camera_index,
        t + 3 * camera_index,
        camera + 3 * camera_index,
        landmark + 3 * landmark_index);
}

void BAL::solve()
{
    // 配置求解器
    ceres::Solver::Options options;
    // 打印到cout
    options.minimizer_progress_to_stdout = true;
    // 增量方程如何来计算
    options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
    // 增量方程如何来计算
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations = 100;
    // 求解器
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
    // for (int i = 0; i < camera_num; i++)
    // {
    //     std::cout << "camare " << i << " estimated angleAxis = ";
    //     for (int j = 0; j < 3; j++)
    //     {
    //         std::cout << angleAxis[3 * i + j] << " ";
    //     }
    //     std::cout << std::endl;
    //     std::cout << "camare " << i << " estimated translate = ";
    //     for (int j = 0; j < 3; j++)
    //     {
    //         std::cout << t[3 * i + j] << " ";
    //     }
    //     std::cout << std::endl;

    //     std::cout << "camare " << i << " estimated f,k1,k2 = ";
    //     for (int j = 0; j < 3; j++)
    //     {
    //         std::cout << camera[3 * i + j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
    std::cout << std::endl;
}

rust::Vec<double> BAL::get_angle_axis(int camera_index)
{
    rust::Vec<double> slice;
    auto ptr = angleAxis + 3 * camera_index;
    slice.push_back(ptr[0]);
    slice.push_back(ptr[1]);
    slice.push_back(ptr[2]);
    return slice;
}

rust::Vec<double> BAL::get_translate(int camera_index)
{
    rust::Vec<double> slice;
    auto ptr = t + 3 * camera_index;
    slice.push_back(ptr[0]);
    slice.push_back(ptr[1]);
    slice.push_back(ptr[2]);
    return slice;
}

// 相机内参
rust::Vec<double> BAL::get_camera_inter(int camera_index)
{
    rust::Vec<double> slice;
    auto ptr = camera + 3 * camera_index;
    slice.push_back(ptr[0]);
    slice.push_back(ptr[1]);
    slice.push_back(ptr[2]);
    return slice;
}

rust::Vec<double> BAL::get_camera_args(int camera_index)
{
    rust::Vec<double> slice;
    auto r = angleAxis + 3 * camera_index;
    auto tran = t + 3 * camera_index;
    auto c = camera + 3 * camera_index;
    slice.push_back(r[0]);
    slice.push_back(r[1]);
    slice.push_back(r[2]);
    slice.push_back(tran[0]);
    slice.push_back(tran[1]);
    slice.push_back(tran[2]);
    slice.push_back(c[0]);
    slice.push_back(c[1]);
    slice.push_back(c[2]);
    return slice;
}

rust::Vec<double> BAL::get_landmark(int landmark_index) const
{
    rust::Vec<double> slice;
    auto ptr = landmark + 3 * landmark_index;
    slice.push_back(ptr[0]);
    slice.push_back(ptr[1]);
    slice.push_back(ptr[2]);
    return slice;
}

void BAL::set_angle_axis(int camera_index, double x, double y, double z)
{
    auto ptr = angleAxis + 3 * camera_index;
    ptr[0] = x;
    ptr[1] = y;
    ptr[2] = z;
}

void BAL::set_translate(int camera_index, double x, double y, double z)
{
    auto ptr = t + 3 * camera_index;
    ptr[0] = x;
    ptr[1] = y;
    ptr[2] = z;
}

void BAL::set_camera_inter(int camera_index, double f, double k1, double k2)
{
    auto ptr = camera + 3 * camera_index;
    ptr[0] = f;
    ptr[1] = k1;
    ptr[2] = k2;
}

void BAL::set_landmark(int landmark_index, double x, double y, double z)
{
    auto ptr = landmark + 3 * landmark_index;
    ptr[0] = x;
    ptr[1] = y;
    ptr[2] = z;
}
