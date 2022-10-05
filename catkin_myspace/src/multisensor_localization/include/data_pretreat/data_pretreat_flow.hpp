/*
 * @Description:传感器数据预处理
 * @Author: Robotics gang
 * @note  modified from Ren Qian
 * @Date: 2022-10-02
 */

#ifndef DATA_PRETREAT_FLOW_HPP_
#define DATA_PRETREAT_FLOW_HPP_

// ros库函数
#include <ros/ros.h>
#include <ros/package.h>
//第三方库
#include <yaml-cpp/yaml.h>
//订阅器
#include "../subscriber/cloud_subscriber.hpp"
#include "../subscriber/gnss_subscriber.hpp"
#include "../subscriber/imu_subscriber.hpp"
#include "../subscriber/velocity_subscriber.hpp"
//发布器
#include "../publisher/cloud_publisher.hpp"
#include "../publisher/odometry_publisher.hpp"
#include "../publisher/origin_publisher.hpp"
//模块
// TODO LIST:畸变矫正 但这里似乎是有问题

namespace multisensor_localization
{

    class DataPretreatFlow
    {
    public:
        DataPretreatFlow(ros::NodeHandle &nh);
        bool Run();

    private:
        bool ReadData();
        bool InitCalibration();
        bool InitGNSS();
        bool HasData();
        bool ValidData();
        bool TransformData();
        bool PublishData();

    private:
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<ImuSubscriber> imu_sub_ptr_;
        std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
        std::shared_ptr<GnssSubscriber> gnss_sub_ptr_;

        std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
        std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
        std::shared_ptr<OriginPublisher> origin_pub_ptr_;

        std::deque<CloudData> cloud_data_buff_;
        std::deque<ImuData> imu_data_buff_;
        std::deque<VelocityData> velocity_data_buff_;
        std::deque<GnssData> gnss_data_buff_;

        Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();

        YAML::Node config_node_;

        CloudData current_cloud_data_;
        ImuData current_imu_data_;
        VelocityData current_velocity_data_;
        GnssData current_gnss_data_;
    };

} // namespace multisensor_localization

#endif