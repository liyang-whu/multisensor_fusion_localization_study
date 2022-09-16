#ifndef _ODOM_PUBLISHER_H
#define _ODOM_PUBLISHER_H

#include "../../head.hpp"

namespace multisensor_localization
{
    class OdomPublisher
    {
    public:
        OdomPublisher(ros::NodeHandle &nh, string topic_name, string base_frame_id, string child_frame_id, int buff_size);
        OdomPublisher() = default;

        void Publish(const Eigen::Matrix4f &transform_matrix);

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        nav_msgs::Odometry odom_;
    };
}
#endif