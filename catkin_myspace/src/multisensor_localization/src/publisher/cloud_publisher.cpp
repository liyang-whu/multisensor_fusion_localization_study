/*
 * @Description:点云发布器
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#include "../../include/publisher/cloud_publisher.hpp"

namespace multisensor_localization
{
    /**
     * @brief 点云发布初始化
     * @note
     * @todo
     **/
    CloudPublisher::CloudPublisher(ros::NodeHandle &nh,
                                   std::string topic_name,
                                   std::string frame_id,
                                   size_t buff_size)
        : nh_(nh), frame_id_(frame_id)
    {
        publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
    }

    /**
     * @brief 点云发布消息
     * @note 重载一
     * @todo
     **/
    void CloudPublisher::Publish(CloudData::CLOUD_PTR &cloud_ptr_input, double time)
    {
        ros::Time ros_time((float)time);
        PublishData(cloud_ptr_input, ros_time);
    }

    /**
     * @brief 点云发布消息
     * @note 重载一 取外参时间戳
     * @todo
     **/
    void CloudPublisher::Publish(CloudData::CLOUD_PTR &cloud_ptr_input)
    {
        ros::Time time = ros::Time::now();
        PublishData(cloud_ptr_input, time);
    }

    /**
     * @brief 点云发布消息
     * @note 重载二 取发布时刻时间戳
     * @todo
     **/
    void CloudPublisher::PublishData(CloudData::CLOUD_PTR &cloud_ptr_input, ros::Time time)
    {
        sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);

        cloud_ptr_output->header.stamp = time;
        cloud_ptr_output->header.frame_id = frame_id_;
        publisher_.publish(*cloud_ptr_output);
    }

    /**
     * @brief 检查是否被订阅
     * @note
     * @todo
     **/
    bool CloudPublisher::HasSubscribers()
    {
        return publisher_.getNumSubscribers() != 0;
    }
} // namespace lidar_localization