/*
 * @Description:订阅imu消息
 * @Author: Robotic Gang
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#ifndef ODOMETRY_SUBSCRIBER_HPP_
#define ODOMETRY_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include <deque>
#include "../sensor_data/pose_data.hpp"
#include <nav_msgs/Odometry.h>


namespace multisensor_localization
{

    class OdometrySubscriber
    {
        public:
        OdometrySubscriber(ros::NodeHandle&nh,std::string topic_name,size_t buff_size);
        OdometrySubscriber()=default;
       void ParseData(std::deque<PoseData>&pose_data_buff);

        private:
        void MsgCallback(const nav_msgs::OdometryConstPtr &odom_msg_ptr);

        private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<PoseData> new_pose_data_buff_;
    };

} // namespace multisensor_localization

#endif