#include "../../head.hpp"
#include "../../include/subscriber/imu_subscriber.hpp"

namespace multisensor_localization
{
    // imu订阅器初始化
    ImuSubscriber::ImuSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size) : nh_(nh)
    {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &ImuSubscriber::MsgCallback, this);
    }

    void ImuSubscriber::MsgCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr)
    {
        ImuData imu_data;
        /*拷贝时间戳*/
        imu_data.time_stamp_ = imu_msg_ptr->header.stamp.toSec();
        /*拷贝加速度*/
        imu_data.linear_acceleration_.x = imu_msg_ptr->linear_acceleration.x;
        imu_data.linear_acceleration_.y = imu_msg_ptr->linear_acceleration.y;
        imu_data.linear_acceleration_.z = imu_msg_ptr->linear_acceleration.z;
        /*拷贝角速度*/
        imu_data.angular_velocity_.x = imu_msg_ptr->angular_velocity.x;
        imu_data.angular_velocity_.y = imu_msg_ptr->angular_velocity.y;
        imu_data.angular_velocity_.z = imu_msg_ptr->angular_velocity.z;
        /*拷贝四元数*/
        imu_data.orientation_.x = imu_msg_ptr->orientation.x;
        imu_data.orientation_.y = imu_msg_ptr->orientation.y;
        imu_data.orientation_.z = imu_msg_ptr->orientation.z;
        imu_data.orientation_.w = imu_msg_ptr->orientation.w;
        /*加入队列*/
        new_imu_data_.push_back(imu_data);
    }

    void ImuSubscriber::ParseData(deque<ImuData> &imu_data_buff)
    {
        /*模拟类似串口缓冲区的机制*/
        if (new_imu_data_.size() > 0)
        {
            imu_data_buff.insert(imu_data_buff.end(),
                                 new_imu_data_.begin(),
                                 new_imu_data_.end());

            new_imu_data_.clear();
        }
    }

} // multisensor_localization