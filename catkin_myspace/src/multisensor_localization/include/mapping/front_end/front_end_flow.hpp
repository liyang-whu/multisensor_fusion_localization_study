/*
 * @Description:
 * @Author: Robotic Gang
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#ifndef FRONT_END_FRONT_END_FLOW_HPP_
#define FRONT_END_FRONT_END_FLOW_HPP_

#include <ros/ros.h>

#include "../../subscriber/cloud_subscriber.hpp"
#include "../../publisher/odometry_publisher.hpp"
#include <glog/logging.h>

namespace multisensor_localization
{
    class FrontEndFlow
    {
    public:
        FrontEndFlow(ros::NodeHandle &nh);

        bool Run();

    private:
        bool ReadData();
        bool HasData();
        bool ValidData();
        bool UpdateLaserOdometry();//这个是变换量
        bool PublishData();

    private:
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
        //std::shared_ptr<FrontEnd> front_end_ptr_;

        std::deque<CloudData> cloud_data_buff_;

        CloudData current_cloud_data_;

        Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
    };
}

#endif