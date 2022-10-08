#include "./graph_optimizer_interface.hpp"

#ifndef G2O_OPTIMIZER_HPP_
#define G2O_OPTIMIZER_HPP_

#include <g2o/core/g2o_core_api.h>

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
        /*添加鲁棒核、节点、边、*/
        virtual void SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size) override;
        virtual void AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix) override;
        //!为什么这里是两个vertex_index???
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

    };

} // namespace multisensor_localization

#endif