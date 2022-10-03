/*
 * @Description: 点云发布器
 * @Author: Robotics gang
 * @note  modified from Ren Qian
 * @Date: 2022-10-02
 */

#ifndef ORIGIN_PUBLISHER_HPP_
#define ORIGIN_PUBLISHER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "../sensor_data/gnss_data.hpp"

namespace multisensor_localization
{
    class OriginPublisher
    {
    public:
        OriginPublisher(ros::NodeHandle &nh, std::string topic_name, size_t buff_size, std::string frame_id);
        OriginPublisher() = default;

        void Publish(GnssData &gnss_input);

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        sensor_msgs::NavSatFix origin_;
        std::string frame_id_;
    };
}

#endif