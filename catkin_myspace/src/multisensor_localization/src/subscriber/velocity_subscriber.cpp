/*
 * @Description:速度订阅器
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#include "../../include/subscriber/velocity_subscriber.hpp"

namespace multisensor_localization
{

    /**
     * @brief 速度订阅初始化
     * @note
     * @todo
     **/
    VelocitySubscriber::VelocitySubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size)
        : nh_(nh)
    {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &VelocitySubscriber::MsgCallback, this);
    }

    /**
     * @brief 速度订阅回调函数
     * @note
     * @todo
     **/
    void VelocitySubscriber::MsgCallback(const geometry_msgs::TwistStampedConstPtr &twist_msg_ptr)
    {
        VelocityData velocity_data;
        velocity_data.time_stamp_ = twist_msg_ptr->header.stamp.toSec();

        velocity_data.linear_velocity_.x = twist_msg_ptr->twist.linear.x;
        velocity_data.linear_velocity_.y = twist_msg_ptr->twist.linear.y;
        velocity_data.linear_velocity_.z = twist_msg_ptr->twist.linear.z;

        velocity_data.angular_velocity_.x = twist_msg_ptr->twist.angular.x;
        velocity_data.angular_velocity_.y = twist_msg_ptr->twist.angular.y;
        velocity_data.angular_velocity_.z = twist_msg_ptr->twist.angular.z;

        new_velocity_data_buff_.push_back(velocity_data);
    }

    /**
     * @brief 读取缓冲区数据并清空
     * @note
     * @todo
     **/
    void VelocitySubscriber::ParseData(std::deque<VelocityData> &velocity_data_buff)
    {
        if (new_velocity_data_buff_.size() > 0)
        {
            velocity_data_buff.insert(velocity_data_buff.end(), new_velocity_data_buff_.begin(), new_velocity_data_buff_.end());
            new_velocity_data_buff_.clear();
        }
    }
}