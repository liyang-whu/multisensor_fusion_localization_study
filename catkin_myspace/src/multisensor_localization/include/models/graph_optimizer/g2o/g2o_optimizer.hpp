#include "../graph_optimizer_interface.hpp"

#ifndef G2O_OPTIMIZER_HPP_
#define G2O_OPTIMIZER_HPP_

#include <g2o/core/g2o_core_api.h>
#include  <g2o/core/factory.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/sparse_optimizer.h>

namespace g2o
{

} // namespace g2o

namespace multisensor_localization
{
    class G2oOptimizer : public GrapOptimizerInterface
    {
        G2oOptimizer(const std::string &solver_type = "Im_var");
        /*优化动作*/
        virtual bool Optimize() override;
        /*输入输出数据*/
        virtual bool GetOptimizedPose(std::deque<Eigen::Matrix4f> &Optimized_pose) override;
        virtual bool GetNodeNum() override;
        /*添加鲁棒核*/
        virtual void SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size) override;
        /*添加节点*/
        virtual void AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix) override;
        /*添加边*/
        //!为什么这里是两个vertex_index??? 先验边???
        virtual void AddSe3Edge(int vertex_index1,
                                int vertex_index2,
                                const Eigen::Isometry3d &relative_pose,
                                const Eigen::VectorXd noise) override;
        virtual void AddSe3PriorXYZEdge(int se3_vertex_index,
                                        const Eigen::Vector3d &xyz,
                                        const Eigen::VectorXd noise) override;
        virtual void AddSe3PriorQuaternionEdge(int se3_vertex_index,
                                               const Eigen::Quaterniond &quat,
                                               const Eigen::VectorXd noise) override;
        private:
        /*计算信息矩阵 ps:其实就是权重啦*/
        Eigen::MatrixXd CalculateSe3EdgeInformationMatrix(Eigen::VectorXd noise);
        Eigen::MatrixXd CalculateSe3PriorQuaternionEdgeInformationMatrix(Eigen::VectorXd noise);
        Eigen::MatrixXd CalculateDiagMatrix(Eigen::VectorXd noise);

        g2o::RobustKernelFactory * robust_kernel_factroy_;
        std::unique_ptr<g2o::SparseOptimizer> graph_ptr_;
        std::string robust_kernel_size_;
        bool need_robust_kernel_=false;
    };

} // namespace multisensor_localization

#endif