#include "../../../include/models/registration/ndt_registration.hpp"

namespace multisensor_localization
{
    NdtRegistration::NdtRegistration(const YAML::Node &node)
    {
    }

    NdtRegistration::NdtRegistration(float res, float step_size, float trans_eps, int max_iter)
    {
           
    }

    bool NdtRegistration::SetTarget(const CloudData::CLOUD_PTR &target)
    {
        return true;
    }

    bool NdtRegistration::ScanMatch(
        const CloudData::CLOUD_PTR &cloud_in,
        const CloudData::CLOUD_PTR cloud_out,
        const Eigen::Matrix4f &pose_predict,
        const Eigen::Matrix4f &pose_result)
    {
          return true;
    }

}