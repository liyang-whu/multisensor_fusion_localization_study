/*
 * @Description: 点云发布器
 * @Author: Robotics gang
 * @note  modified from Ren Qian
 * @Date: 2022-10-02
 */

#ifndef  ODOM_PUBLISHER_HPP_
#define ODOM_PUBLISHER_HPP_

#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>


namespace multisensor_localization
{
    class OdomPublisher
    {
    public:
        OdomPublisher(ros::NodeHandle &nh, std::string topic_name, std::string base_frame_id, std::string child_frame_id, int buff_size);
        OdomPublisher() = default;

        void Publish(const Eigen::Matrix4f &transform_matrix);

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        nav_msgs::Odometry odom_;
    };
}
#endif