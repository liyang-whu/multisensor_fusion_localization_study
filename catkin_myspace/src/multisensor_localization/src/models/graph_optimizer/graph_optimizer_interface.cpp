#include "../../../include/models/graph_optimizer/graph_optimizer_interface.hpp"

namespace multisensor_localization
{

    void GrapOptimizerInterface::SetMaxIterationsNum(int max_iterations_num)
    {
        max_iterations_num_=max_iterations_num;
    }

} // namespace multisensor_localization