/*
 * @Description: 发布点云消息
 * @Author: Robotics gang
 * @note  modified from Ren Qian
 * @Date: 2022-10-02
 */
#include "../../include/publisher/cloud_publisher.hpp"

namespace multisensor_localization
{
    /**
     * @brief  点云发布器初始化
     * @note
     * @todo
     **/
    CloudPublisher::CloudPublisher(ros::NodeHandle &nh,
                                   std::string topic_name,
                                   size_t buff_size,
                                   std::string frame_id) : nh_(nh), frame_id_(frame_id)
    {
        publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
    };

    /**
     * @brief  点云发布消息
     * @note
     * @todo
     **/
    void CloudPublisher::Publish(CloudData::CLOUD_PTR cloud_input_ptr)
    {
        sensor_msgs::PointCloud2Ptr cloud_output_ptr(new sensor_msgs::PointCloud2());
        /*PCL类型转换ROS类型*/
        pcl::toROSMsg(*cloud_input_ptr, *cloud_output_ptr);
        /*设置时间戳及id*/
        cloud_output_ptr->header.stamp = ros::Time::now();
        cloud_output_ptr->header.frame_id = frame_id_;
        publisher_.publish(*cloud_output_ptr);
    }
}