/*
 * @Description:传感器数据预处理
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#include "../../../include/mapping/front_end/front_end_flow.hpp"

namespace multisensor_localization
{
    /**
     * @brief 前端流程控制初始化
     * @note 订阅点云信息 发布激光里程计
     * @todo
     **/
    FrontEndFlow::FrontEndFlow(ros::NodeHandle &nh)
    {
        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
        laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_odom", "/map", "/lidar", 100);

        front_end_ptr_ = std::make_shared<FrontEnd>();
    }

    /**
     * @brief 前端流程运行
     * @note
     * @todo
     **/
    bool FrontEndFlow::Run()
    {
        if (!ReadData())
            return false;

        while (HasData())
        {
            if (!ValidData())
                continue;

            if (UpdateLaserOdometry())
            {
                PublishData();
            }
        }

        return true;
    }

    /**
     * @brief 数据读取
     * @note 读取点云数据
     * @todo
     **/
    bool FrontEndFlow::ReadData()
    {
        cloud_sub_ptr_->ParseData(cloud_data_buff_);
        return true;
    }

    /**
     * @brief 检查是否存在数据
     * @note
     * @todo
     **/
    bool FrontEndFlow::HasData()
    {
        return cloud_data_buff_.size() > 0;
    }

    /**
     * @brief 提取有效数据
     * @note
     * @todo
     **/
    bool FrontEndFlow::ValidData()
    {
        current_cloud_data_ = cloud_data_buff_.front();
        cloud_data_buff_.pop_front();

        return true;
    }

    /**
     * @brief 更新激光里程计数据
     * @note
     * @todo
     **/
    bool FrontEndFlow::UpdateLaserOdometry()
    {
        static bool odometry_inited = false;
        if (!odometry_inited)
        {
            odometry_inited = true;
            front_end_ptr_->SetInitPose(Eigen::Matrix4f::Identity());//不再需要gnss提供初值
            return front_end_ptr_->UpdateOdometry(current_cloud_data_, laser_odometry_);
        }
        return front_end_ptr_->UpdateOdometry(current_cloud_data_, laser_odometry_);
    }

    /**
     * @brief 激光里程计算发布到后端优化
     * @note
     * @todo
     **/
    bool FrontEndFlow::PublishData()
    {
        laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time_stamp_);

        return true;
    }
}