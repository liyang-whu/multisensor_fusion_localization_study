#ifndef VELOCITY_SUBSCRIBER_H
#define VELOCITY_SUBSCRIBER_H

#include "../../head.hpp"
#include "../sensor_data/velocity_data.hpp"

namespace multisensor_localization
{

    class VelocitySubscriber
    {
    public:
        VelocitySubscriber(ros::NodeHandle &nh, string topic_name, size_t buff_size);
        VelocitySubscriber() = default;
        void ParseData(deque<VelocityData> &velocity_data_buff);

    private:
        void MsgCallback(const geometry_msgs::TwistStampedConstPtr &twist_msg_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        deque<VelocityData> new_velocity_data_;
    };

} // namespace multisensor_localization

#endif