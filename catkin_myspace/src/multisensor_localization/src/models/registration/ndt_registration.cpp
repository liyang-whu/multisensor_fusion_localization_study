/*
 * @Description: voxel filter 模块实现
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:53:20
 */
#include "../../../include/models/registration/ndt_registration.hpp"
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>

namespace multisensor_localization
{

    /**
     * @brief 点云匹配初始化(参数文件配置)
     * @note
     * @todo
     **/
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
     * @brief 点云匹配初始化(形式参数配置)
     * @note
     * @todo
     **/
    NdtRegistration::NdtRegistration(float res, float step_size, float trans_eps, int max_iter)
        : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>())
    {

        SetRegistrationParam(res, step_size, trans_eps, max_iter);
    }

    /**
     * @brief 设置匹配参数
     * @note
     * @todo
     **/
    bool NdtRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter)
    {
        ndt_ptr_->setResolution(res);
        ndt_ptr_->setStepSize(step_size);
        ndt_ptr_->setTransformationEpsilon(trans_eps);
        ndt_ptr_->setMaximumIterations(max_iter);

        return true;
    }

    /**
     * @brief 设置目标点云
     * @note 向target匹配
     * @todo
     **/
    bool NdtRegistration::SetInputTarget(const CloudData::CLOUD_PTR &input_target)
    {
        ndt_ptr_->setInputTarget(input_target);

        return true;
    }
    bool NdtRegistration::ScanMatch(const CloudData::CLOUD_PTR &input_cloud_ptr,
                                    const Eigen::Matrix4f &predict_pose,
                                    CloudData::CLOUD_PTR &result_cloud_ptr,
                                    Eigen::Matrix4f &result_pose)
    {
        ndt_ptr_->setInputSource(input_cloud_ptr);
        ndt_ptr_->align(*result_cloud_ptr, predict_pose);
        result_pose = ndt_ptr_->getFinalTransformation();

        return true;
    }

}