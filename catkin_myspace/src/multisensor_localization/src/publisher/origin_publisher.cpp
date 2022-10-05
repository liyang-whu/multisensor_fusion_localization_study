/*
 * @Description:点云发布器
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#include "../../include/publisher/origin_publisher.hpp"

namespace multisensor_localization
{

    /**
     * @brief 原点发布器初始化
     * @note
     * @todo
     **/
    OriginPublisher::OriginPublisher(ros::NodeHandle &nh, std::string topic_name, size_t buff_size, std::string frame_id) \
    : nh_(nh), frame_id_(frame_id)
    {
        publisher_ = nh_.advertise<sensor_msgs::NavSatFix>(topic_name, buff_size);
    };

    /**
     * @brief 发布坐标起点
     * @note
     * @todo
     **/
    void OriginPublisher::Publish(GnssData &gnss_input)
    {
        origin_.header.frame_id = frame_id_;
        origin_.latitude = gnss_input.latitude_;
        origin_.longitude = gnss_input.longitude_;
        origin_.altitude = gnss_input.status_;
        origin_.status.service = gnss_input.service_;

        publisher_.publish(origin_);
    }

}