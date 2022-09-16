
#include "../../head.hpp"
#include "../../include/publisher/origin_publisher.hpp"

namespace multisensor_localization
{

    //原点发布器初始化
    OriginPublisher::OriginPublisher(ros::NodeHandle &nh, string topic_name, size_t buff_size, string frame_id) : nh_(nh), frame_id_(frame_id)
    {
        publisher_ = nh_.advertise<sensor_msgs::NavSatFix>(topic_name, buff_size);
    };

    //原点发布
    void OriginPublisher::Publish(GnssData &gnss_input)
    {
        origin_.header.frame_id = frame_id_;
        origin_.latitude = gnss_input.latitude_;
        origin_.longitude = gnss_input.longtitude_;
        origin_.altitude = gnss_input.status_;
        origin_.status.service = gnss_input.service_;

        cout << "has sent origin" << endl;
        publisher_.publish(origin_);
    }

}