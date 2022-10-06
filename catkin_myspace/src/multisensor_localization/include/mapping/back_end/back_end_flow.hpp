/*
 * @Description: back end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:31:22
 */

#ifndef BACK_END_FRONT_END_FLOW_HPP_
#define BACK_END_FRONT_END_FLOW_HPP_

#include <ros/ros.h>

#include "../../../include/subscriber/cloud_subscriber.hpp"
#include "../../../include/subscriber/gnss_subscriber.hpp"
#include "../../../include/subscriber/odometry_subscriber.hpp"
#include "../../../include/publisher/odometry_publisher.hpp"

namespace multisensor_localization
{
    class BackEndFlow
    {
    public:
        BackEndFlow(ros::NodeHandle &nh);
        bool Run();

    private:
        bool ReadData();
        bool HasData();
        bool ValidData();

        bool UpdateBackEnd();
        bool SaveTrajectory();

    private:
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> gnss_odom_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> laser_odom_sub_ptr_;

    std::shared_ptr<OdometryPublisher> transformed_odom_pub_ptr_;

    };
}

#endif
