/*
 * @Description:传感器数据预处理
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#include "../../../include/mapping/back_end/back_end_flow.hpp"
#include "../../../include/sensor_data/cloud_data.hpp"
#include "../../../include/subscriber/odometry_subscriber.hpp"
#include "../../../include/publisher/odometry_publisher.hpp"

namespace multisensor_localization
{
    /**
     * @brief 后面端流程控制初始化
     * @note
     * @todo
     **/
    BackEndFlow::BackEndFlow(ros::NodeHandle &nh)
    {
        /*话题订阅*/
        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 1e5);
        gnss_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 1e5);
        laser_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/laser_odom", 1e5);

        /*话题发布*/
        transformed_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/transformed_odom", "/map", "/lidar", 100);

    }

    /**
     * @brief 后端流程控制初始化
     * @note 订阅点云信息 发布激光里程计
     * @todo
     **/
    bool BackEndFlow::Run()
    {

    }

} // namespace multisensor_localization
