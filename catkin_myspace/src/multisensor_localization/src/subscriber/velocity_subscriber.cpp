#include "../../head.hpp"
#include "../../include/subscriber/velocity_subscriber.hpp"

namespace multisensor_localization
{

    /**
     * @brief 速度计算初始化
     * @note
     * @todo
     **/
    VelocitySubscriber::VelocitySubscriber(ros::NodeHandle &nh, string topic_name, size_t buff_size)
        : nh_(nh)
    {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &VelocitySubscriber::MsgCallback, this);
    }

    /**
     * @brief 回调函数
     * @note
     * @todo
     **/
    void VelocitySubscriber::MsgCallback(const geometry_msgs::TwistStampedConstPtr &twist_msg_ptr)
    {
        VelocityData velocity_data;

        /*数据格式转换*/
        velocity_data.time_stamp_ = twist_msg_ptr->header.stamp.toSec();

        velocity_data.linear_velocity_.x = twist_msg_ptr->twist.linear.x;
        velocity_data.linear_velocity_.y = twist_msg_ptr->twist.linear.y;
        velocity_data.linear_velocity_.z = twist_msg_ptr->twist.linear.z;

        velocity_data.angular_velocity_.x = twist_msg_ptr->twist.angular.x;
        velocity_data.angular_velocity_.y = twist_msg_ptr->twist.angular.y;
        velocity_data.angular_velocity_.z = twist_msg_ptr->twist.angular.z;

        new_velocity_data_.push_back(velocity_data);
    }

    /**
     * @brief 读取缓冲区数据
     * @note
     * @todo
     **/
    void VelocitySubscriber::ParseData(deque<VelocityData> &velocity_data_buff)
    {
        if (new_velocity_data_.size() > 0)
        {
            velocity_data_buff.insert(velocity_data_buff.end(),
                                      new_velocity_data_.begin(),
                                      new_velocity_data_.end());
        }
    }

}