/*
 * @Description:点云发布器
 * @Author: Robotic Gang
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#ifndef KEY_FRAME_PUBLISHER_HPP_
#define KEY_FRAME_PUBLISHER_HPP_

#include <deque>
#include <ros/ros.h>

#include "../../include/sensor_data/key_frame.hpp"

namespace multisensor_localization
{

    class KeyFramePublisher
    {
        public:
        KeyFramePublisher(ros::NodeHandle &nh,
                          std::string topic_name,
                          std::string frame_id,
                          int buff_size);
        KeyFramePublisher() = default;
        void Publish(KeyFrame &key_frame);
        bool HasSubscribers();

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_ = "";
    };

} // namespace name

#endif