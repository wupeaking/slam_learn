## slam十四讲的rust代码实现

### pose_estimation
    主要是相机的位置估计 有2d-2d 2d-3d 3d-3d
    2d-2d: 通过特征点匹配计算本质矩阵，然后通过本质矩阵分解得到R t
    2d-3d: PnP 和 BA 使用OPECV调用库函数(PNP) 和手写的GN优化 和ceres优化(BA)
    3d-3d: ICP

### lk_flow
    光流法的实现 opecv的库调用和手写gn的实现

### bal_optimization
    用ceres实现的BAL数据集的优化

### pose_graph_optimization
    用ceres实现的位姿图优化 用g2o生成的示例数据