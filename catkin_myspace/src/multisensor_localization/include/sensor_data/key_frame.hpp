/*
 * @Description:
 * @Author: Robotic Gang
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#ifndef KEY_FRAME_HPP_
#define KEY_FRAME_HPP_

#include <Eigen/Dense>

namespace multisensor_localization
{

    class KeyFrame
    {
    public:
        Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();
        double time_stamp_ = 0.0;
        unsigned int index_= 0;

    public:
        Eigen::Quaternionf GetQuaternion();
    };

} // namesapce multisensor_localization

#endif