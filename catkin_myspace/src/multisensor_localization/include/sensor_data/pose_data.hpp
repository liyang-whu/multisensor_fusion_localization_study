/*
 * @Description:订阅imu消息
 * @Author: Robotic Gang
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#ifndef POSE_DATA_HPP_
#define POSE_DATA_HPP_

#include <Eigen/Dense>

namespace multisensor_localization
{

    class PoseData
    {
    public:
        Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();
        double time_stamp_ = 0.0;

        public:
        Eigen::Quaternionf GetQuaternion();
    };

} // namesapce multisensor_localization

#endif
