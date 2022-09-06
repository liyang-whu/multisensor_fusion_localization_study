#ifndef _IMU_SUBSCRIBER_H
#define _IMU_SUBSCRIBER_H

#include "../../head.hpp"
#include "../sensor_data/imu_data.hpp"

namespace multisensor_localization
{
    class ImuSubscriber
    {
    public:
        ImuSubscriber(ros::NodeHandle &nh, string topic_name, size_t buff_size);
        ImuSubscriber() = default;
        void ParseData(deque<ImuData> &imu_data_buff);

    private:
        void MsgCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr);

    private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    deque<ImuData> new_imu_data_;
    };
} // namespace multisensor_localization

#endif