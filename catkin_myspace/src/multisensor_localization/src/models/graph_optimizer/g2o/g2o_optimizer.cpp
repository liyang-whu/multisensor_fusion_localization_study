#include "../../../../include/models/graph_optimizer/g2o/g2o_optimizer.hpp"
#include "../../../../include/debug_tools/debug_tools.hpp"

namespace multisensor_localization
{

    /**
     * @brief 设置求解算法
     * @note
     * @todo
     **/
    G2oOptimizer::G2oOptimizer(const std::string &solver_type)
    {
        graph_optimizer_ptr_.reset(new g2o::SparseOptimizer());

        g2o::OptimizationAlgorithmFactory *solver_factory = g2o::OptimizationAlgorithmFactory::instance();
        g2o::OptimizationAlgorithmProperty solver_property;
        g2o::OptimizationAlgorithm *solver = solver_factory->construct(solver_type, solver_property);
        graph_optimizer_ptr_->setAlgorithm(solver);

        if (!graph_optimizer_ptr_->solver())
        {
            DebugTools::Debug_Error("创建g2o优化器失败");
        }
        robust_kernel_factroy_ = g2o::RobustKernelFactory::instance();
    }

    bool G2oOptimizer::Optimize()
    {
    }

    /**
     * @brief 添加鲁棒核
     * @note 鲁棒核主要用于避免异常点带来的误差
     * @todo
     **/
    void G2oOptimizer::SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size)
    {
        robust_kernel_name_ = robust_kernel_name;
        robust_kernel_size_ = robust_kernel_size;
        need_robust_kernel_ = true;
    }
    /**
     * @brief 添加顶点
     * @note
     * @todo
     **/
    void G2oOptimizer::AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix)
    {
          g2o::VertexSE3 * vertex(new g2o::VertexSE3());
    }

} // namespace multisensor_localization