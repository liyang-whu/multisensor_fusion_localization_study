/*
 * @Description:订阅gnss消息
 * @Author: Robotic Gang
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#ifndef CLOUD_SUBSCRIBER_HPP_
#define CLOUD_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <glog/logging.h>
#include "../sensor_data/gnss_data.hpp"

namespace multisensor_localization
{
  class GnssSubscriber
  {
  public:
    GnssSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
    GnssSubscriber() = default;
    void ParseData(std::deque<GnssData> &gnss_data_buff);

  private:
    void MsgCallback(const sensor_msgs::NavSatFixConstPtr &nav_sat_fix_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<GnssData> new_gnss_data_buff_;
  };
}
#endif