/*
 * @Description:点云发布器
 * @Author: Robotic Gang
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#ifndef KEY_FRAMES_HPP_
#define KEY_FRAMES_HPP_

#include <string>
#include <deque>
#include <ros/ros.h>

#include "../../include/sensor_data/key_frame.hpp"

namespace multisensor_localization
{
  class KeyFramesPublisher
  {
  public:
    KeyFramesPublisher(ros::NodeHandle &nh,
                       std::string topic_name,
                       std::string frame_id,
                       int buff_size);
    KeyFramesPublisher() = default;

    void Publish(const std::deque<KeyFrame> &key_frames_buff);

    bool HasSubscribers();

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";
  };

} // namespace name

#endif