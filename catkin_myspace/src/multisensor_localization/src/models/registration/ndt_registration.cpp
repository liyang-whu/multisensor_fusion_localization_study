#include "../../../include/models/registration/ndt_registration.hpp"

namespace multisensor_localization
{

    /**
   @brief DNT初始化(YAML读参)
  */
    NdtRegistration::NdtRegistration(const YAML::Node &node)
        : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>())
    {
        float res = node["res"].as<float>();
        float step_size = node["step_size"].as<float>();
        float trans_eps = node["trans_eps"].as<float>();
        int max_iter = node["max_iter"].as<int>();

        SetRegistrationParam(res, step_size, trans_eps, max_iter);
    }

    /**
      @brief DNT初始化(函数传参)
     */
    NdtRegistration::NdtRegistration(float res, float step_size, float trans_eps, int max_iter)
        : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>())
    {
        SetRegistrationParam(res, step_size, trans_eps, max_iter);
    }

    /**
        @brief DNT 设置目标点云
       */
    bool NdtRegistration::SetTarget(const CloudData::CLOUD_PTR &input_target)
    {
        ndt_ptr_->setInputTarget(input_target);
        return true;
    }

    /**
        @brief DNT 点云匹配
       */
    bool NdtRegistration::ScanMatch(
        const CloudData::CLOUD_PTR &cloud_in_ptr,
        const Eigen::Matrix4f &pose_in,
        CloudData::CLOUD_PTR cloud_out_ptr,
        Eigen::Matrix4f &pose_out)
    {
        ndt_ptr_->setInputSource(cloud_in_ptr);
        ndt_ptr_->align(*cloud_out_ptr, pose_in);
        pose_out = ndt_ptr_->getFinalTransformation();

        return true;
    }

    /**
    @brief DNT 参数设置
   */
    bool NdtRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter)
    {
        ndt_ptr_->setResolution(res);
        ndt_ptr_->setStepSize(step_size);
        ndt_ptr_->setTransformationEpsilon(trans_eps);
        ndt_ptr_->setMaximumIterations(max_iter);

        LOG(INFO) << endl
                  << fontColorYellow << "DNT匹配参数" << fontColorReset << endl
                  << fontColorBlue
                  << "res\t\t" << res << endl
                  << "step_size\t" << step_size << endl
                  << "trans_eps\t" << trans_eps << endl
                  << "max_iter\t" << max_iter << endl
                  << fontColorReset << endl;

        return true;
    }

}