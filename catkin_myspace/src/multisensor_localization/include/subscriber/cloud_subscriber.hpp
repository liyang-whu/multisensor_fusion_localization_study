/*
 * @Description: 接收点云消息
 * @Author: Robotics gang
 * @note  modified from Ren Qian
 * @Date: 2022-10-02
 */

#ifndef  CLOUD_SUBSCRIBER_HPP_
#define CLOUD_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include <deque>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "../sensor_data/cloud_data.hpp"



namespace multisensor_localization
{
    class CloudSubscriber
    {
    public:
        CloudSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
        CloudSubscriber() = default;
        void ParseData(std::deque<CloudData> &cloud_data_buff);

    private:
        void MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<CloudData> new_cloud_data_;
    };
}//namespace multisensor_localization

#endif