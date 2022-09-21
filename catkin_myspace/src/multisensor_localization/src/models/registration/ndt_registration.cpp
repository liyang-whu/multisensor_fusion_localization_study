#include "../../../include/models/registration/ndt_registration.hpp"

namespace multisensor_localization
{
    NdtRegistration::NdtRegistration(const YAML::Node &node)
        : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>())
    {
        float res = node["res"].as<float>();
        float step_size = node["step_size"].as<float>();
        float trans_eps = node["trans_eps"].as<float>();
        int max_iter = node["max_iter"].as<int>();

        SetRegistrationParam(res, step_size, trans_eps, max_iter);
    }

    NdtRegistration::NdtRegistration(float res, float step_size, float trans_eps, int max_iter)
        : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>())
    {
        SetRegistrationParam(res, step_size, trans_eps, max_iter);
    }

    bool NdtRegistration::SetTarget(const CloudData::CLOUD_PTR &input_target)
    {
        ndt_ptr_->setInputTarget(input_target);
        return true;
    }

    bool NdtRegistration::ScanMatch(
        const CloudData::CLOUD_PTR &cloud_in_ptr,
         CloudData::CLOUD_PTR cloud_out_ptr,
        const Eigen::Matrix4f &pose_in,
         Eigen::Matrix4f &pose_out)
    {
        ndt_ptr_->setInputSource(cloud_in_ptr);
        ndt_ptr_->align(*cloud_out_ptr,pose_in);
        pose_out=ndt_ptr_->getFinalTransformation();

        return true;
    }

    bool NdtRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter)
    {
        ndt_ptr_->setResolution(res);
        ndt_ptr_->setStepSize(step_size);
        ndt_ptr_->setTransformationEpsilon(trans_eps);
        ndt_ptr_->setMaximumIterations(max_iter);

        LOG(INFO) << "NDT的匹配参数" << endl;
        LOG(INFO) << "resolution: " << res << endl;
        LOG(INFO) << "step_size: " << step_size << endl;
        LOG(INFO) << "trans_eps: " << trans_eps << endl;
        LOG(INFO) << "max_iter: " << max_iter << endl;

        return true;
    }

}