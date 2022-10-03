/*
 * @Description: 点云发布器
 * @Author: Robotics gang
 * @note  modified from Ren Qian
 * @Date: 2022-10-02
 */

#ifndef   CLOUD_PUBLISHER_HPP_
#define  CLOUD_PUBLISHER_HPP_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "../sensor_data/cloud_data.hpp"

namespace multisensor_localization
{

    class CloudPublisher
    {
    public:
        CloudPublisher(ros::NodeHandle &nh, std::string topic_name, size_t buff_size,std::string frame_id);
        CloudPublisher() = default;
        void Publish(CloudData::CLOUD_PTR cloud_input_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_;
    };

} // namespace multisensor_localization

#endif