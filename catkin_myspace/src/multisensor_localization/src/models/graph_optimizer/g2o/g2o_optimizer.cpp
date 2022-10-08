#include "../../../../include/models/graph_optimizer/g2o/g2o_optimizer.hpp"

namespace multisensor_localization
{

    G2oOptimizer::G2oOptimizer(const std::string &solver_type)
    {
        g2o::OptimizationAlgorithmFactory *solver_factory =g2o::OptimizationAlgorithmFactory::instance();
        g2o::OptimizationAlgorithmProperty solver_property;
        g2o::OptimizationAlgorithm *solver=solver_factory->construct(solver_type,solver_property);

    }

} // namespace multisensor_localization