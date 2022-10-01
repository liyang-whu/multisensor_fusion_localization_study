#ifndef DISTORTION_CORRECTION_HPP
#define DISTORTION_CORRECTION_HPP

//通用头文件
#include "../../../head.hpp"
#include "../../sensor_data/velocity_data.hpp"
#include "../../sensor_data/cloud_data.hpp"

namespace multisensor_localization
{
class DistortionCorrection
{
public:
    void SetMotionParam(float scan_period,VelocityData velocity_data);
    bool AdjustCloud(CloudData::CLOUD_PTR &input_cloud_ptr, CloudData::CLOUD_PTR &out_put_ptr);
   
private:
 inline Eigen::Matrix3f UpdateMatrix(float real_time);
 private:
    float scan_period_;
    Eigen::Vector3f linear_velocity_;
    Eigen::Vector3f angular_velocity_;
};
}//namespace multisensor_localization



#endif
