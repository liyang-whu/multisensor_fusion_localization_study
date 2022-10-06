/*
 * @Description:imu订阅器
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#include "../../include/subscriber/odometry_subscriber.hpp"

namespace multisensor_localization
{
    /**
     * @brief 里程计订阅器初始化
     * @note
     * @todo
     **/
    OdometrySubscriber::OdometrySubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size)
        : nh_(nh)
    {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &OdometrySubscriber::MsgCallback, this);
    }

    /**
     * @brief 里程计回调函数
     * @note
     * @todo
     **/
    void OdometrySubscriber::MsgCallback(const nav_msgs::OdometryConstPtr &odom_msg_ptr)
    {
        /*时间戳拷贝*/
        PoseData pose_data;
        pose_data.time_stamp_ = odom_msg_ptr->header.stamp.toSec();
        /*位移拷贝*/
        pose_data.pose_(0, 3) = odom_msg_ptr->pose.pose.position.x;
        pose_data.pose_(1, 3) = odom_msg_ptr->pose.pose.position.y;
        pose_data.pose_(2, 3) = odom_msg_ptr->pose.pose.position.z;
        /*旋转拷贝*/
        Eigen::Quaternionf q;
        q.x() = odom_msg_ptr->pose.pose.orientation.x;
        q.y() = odom_msg_ptr->pose.pose.orientation.y;
        q.z() = odom_msg_ptr->pose.pose.orientation.z;
        q.w() = odom_msg_ptr->pose.pose.orientation.w;
        pose_data.pose_.block<3, 3>(0, 0) = q.matrix();

        new_pose_data_buff_.push_back(pose_data);
    }


   /**
     * @brief 里程计缓存数据读取
     * @note
     * @todo
     **/
    void OdometrySubscriber::ParseData(std::deque<PoseData> &pose_data_buff)
    {
        if (new_pose_data_buff_.size() > 0)
        {
            pose_data_buff.insert(pose_data_buff.end(), new_pose_data_buff_.begin(), new_pose_data_buff_.end());
            new_pose_data_buff_.clear();
        }
    }

} // namespace multisensor_localization