#ifndef _NDT_REGISTRATION_H
#define _NDT_REGISTRATION_H

//通用头文件
#include "../../../head.hpp"
//所继承的接口
#include "./registration_interface.hpp"

namespace multisensor_localization
{
    class NdtRegistration : public RegistrationInterface
    {
    public:
        NdtRegistration(const YAML::Node &node);
        NdtRegistration(float res, float step_size, float trans_eps, int max_iter);

        bool SetTarget(const CloudData::CLOUD_PTR &target) override;
        bool ScanMatch(
            const CloudData::CLOUD_PTR &cloud_in_ptr,
            const Eigen::Matrix4f &pose_in,
            CloudData::CLOUD_PTR cloud_out_ptr,
            Eigen::Matrix4f &pose_out) override;

    private:
        bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

    private:
        pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
    };
} // multisensor_localization

#endif