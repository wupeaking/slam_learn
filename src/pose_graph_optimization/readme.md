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
// 信息矩阵的上三角元素
// EDGE_SE3:QUAT ID_a ID_b x_ab y_ab z_ab q_x_ab q_y_ab q_z_ab q_w_ab I_11 I_12 I_13 ... I_16 I_22 I_23 ... I_26 ... I_66 // NOLINT
//
// where I_ij is the (i, j)-th entry of the information matrix for the
// measurement. Only the upper-triangular part is stored. The measurement order
// is the delta position followed by the delta orientation.

/// 优化之后的图形展示
/// python3 polt.py --initial_poses data/ori_poses.txt  --optimized_poses data/opt_poses.txt
pip3 install numpy
pip3 install matplotlib
pip3 install basemap