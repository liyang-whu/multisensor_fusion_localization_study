/*
 * @Description:imu发布器
 * @Author: Robotic Gang
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#ifndef IMU_PUBLISHER_HPP_
#define IMU_PUBLISHER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "../sensor_data/imu_data.hpp"

namespace multisensor_localization
{
    class ImuPublisher
    {
    public:
        ImuPublisher(ros::NodeHandle &nh,
                     std::string topic_name,
                     size_t buff_size,
                     std::string frame_id);

        ImuPublisher()=default;
        
    };
} // namespace multisensor_localization

#endif