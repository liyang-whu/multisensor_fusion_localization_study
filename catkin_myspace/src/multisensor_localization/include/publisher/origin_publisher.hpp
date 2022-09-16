#ifndef  _ORIGIN_PUBLISHER_H
#define _ORIGIN_PUBLISHER_H

#include "../../head.hpp"
#include "../sensor_data/gnss_data.hpp"

namespace multisensor_localization
{
   class OriginPublisher
    {
    public:
        OriginPublisher(ros::NodeHandle &nh, string topic_name, size_t buff_size, string frame_id);
        OriginPublisher() = default;

        void Publish(GnssData & gnss_input );

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        sensor_msgs::NavSatFix origin_;
        string frame_id_;
    };
}

#endif