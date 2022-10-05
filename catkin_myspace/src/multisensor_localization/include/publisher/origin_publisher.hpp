#ifndef  _ORIGIN_PUBLISHER_H
#define _ORIGIN_PUBLISHER_H


#include "../sensor_data/gnss_data.hpp"
#include <sensor_msgs/NavSatFix.h>

namespace multisensor_localization
{
   class OriginPublisher
    {
    public:
        OriginPublisher(ros::NodeHandle &nh, std::string topic_name, size_t buff_size, std::string frame_id);
        OriginPublisher() = default;

        void Publish(GnssData & gnss_input );

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        sensor_msgs::NavSatFix origin_;
        std::string frame_id_;
    };
}

#endif