/*
 * @Description:订阅速度消息
 * @Author: Robotic Gang
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#ifndef VELOCITY_SUBSCRIBER_HPP_
#define VELOCITY_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include <glog/logging.h>
#include <geometry_msgs/TwistStamped.h>
#include "../sensor_data/velocity_data.hpp"

namespace multisensor_localization
{
  class VelocitySubscriber
  {
  public:
    VelocitySubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
    VelocitySubscriber() = default;
    void ParseData(std::deque<VelocityData> &velocity_data_buff);

  private:
    void MsgCallback(const geometry_msgs::TwistStampedConstPtr &twist_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<VelocityData> new_velocity_data_buff_;
  };
}
#endif