/*
 * @Description: 发布里程计
 * @Author: Robotics gang
 * @note  modified from Ren Qian
 * @Date: 2022-10-02
 */
#include "../../include/publisher/odom_publisher.hpp"

namespace multisensor_localization
{

    /**
     * @brief  里程计发布器初始化
     * @note
     * @todo
     **/
    OdomPublisher::OdomPublisher(ros::NodeHandle &nh,
                                 std::string topic_name,
                                 std::string base_frame_id,
                                 std::string child_frame_id, int buff_size) : nh_(nh)
    {
        publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
        odom_.header.frame_id = base_frame_id;
        odom_.child_frame_id = child_frame_id;
    }

    /**
     * @brief  里程计发布消息
     * @note
     * @todo
     **/
    void OdomPublisher::Publish(const Eigen::Matrix4f &transform_matrix)
    {
        /*设置时间戳*/
        odom_.header.stamp = ros::Time::now();
        /*赋平移量*/
        odom_.pose.pose.position.x = transform_matrix(0, 3);
        odom_.pose.pose.position.y = transform_matrix(1, 3);
        odom_.pose.pose.position.z = transform_matrix(2, 3);
        /*赋旋转量*/
        Eigen::Quaternionf q;
        q = transform_matrix.block<3, 3>(0, 0);
        odom_.pose.pose.orientation.x = q.x();
        odom_.pose.pose.orientation.y = q.y();
        odom_.pose.pose.orientation.z = q.z();
        odom_.pose.pose.orientation.w = q.w();
        /*里程计发布*/
        publisher_.publish(odom_);
    }

}
