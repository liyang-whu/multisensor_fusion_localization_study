

#ifndef G2O_OPTIMIZER_HPP_
#define G2O_OPTIMIZER_HPP_

#include "../graph_optimizer_interface.hpp"
#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o
{
class VertexSE3;
class VertexPlane;
class VertexPointXYZ;
class EdgeSE3;
class EdgeSE3Plane;
class EdgeSE3PointXYZ;
class EdgeSE3PriorXY;
class EdgeSE3PriorXYZ;
class EdgeSE3PriorVec;
class EdgeSE3PriorQuat;
class RobustKernelFactory;
} // namespace g2o



namespace multisensor_localization
{
    
    class G2oOptimizer : public GrapOptimizerInterface
    {
        public:
        G2oOptimizer(const std::string &solver_type = "Im_var");
        // /*优化动作*/
         bool Optimize() override;
        // /*输出数据*/
        //  bool GetOptimizedPose(std::deque<Eigen::Matrix4f> &Optimized_pose) {}
        //  bool GetNodeNum() override;
        // /*添加鲁棒核*/
         void SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size) override;
        // /*添加节点*/
        void AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix)override;
        // /*添加边*/
        // //!为什么这里是两个vertex_index??? 先验边???
        //  void AddSe3Edge(int vertex_index1,
        //                         int vertex_index2,
        //                         const Eigen::Isometry3d &relative_pose,
        //                         const Eigen::VectorXd noise) {}
        //  void AddSe3PriorXYZEdge(int se3_vertex_index,
        //                                 const Eigen::Vector3d &xyz,
        //                                 const Eigen::VectorXd noise) {}
        //  void AddSe3PriorQuaternionEdge(int se3_vertex_index,
        //                                        const Eigen::Quaterniond &quat,
        //                                        const Eigen::VectorXd noise) {}

    private:
        /*计算信息矩阵 ps:其实就是权重*/
        Eigen::MatrixXd CalculateSe3EdgeInformationMatrix(Eigen::VectorXd noise);
        Eigen::MatrixXd CalculateSe3PriorQuaternionEdgeInformationMatrix(Eigen::VectorXd noise);
        Eigen::MatrixXd CalculateDiagMatrix(Eigen::VectorXd noise);

        g2o::RobustKernelFactory *robust_kernel_factroy_;
        std::unique_ptr<g2o::SparseOptimizer> graph_optimizer_ptr_;

        std::string robust_kernel_name_;
        double robust_kernel_size_;
        bool need_robust_kernel_ = false;
    };

} // namespace multisensor_localization

#endif