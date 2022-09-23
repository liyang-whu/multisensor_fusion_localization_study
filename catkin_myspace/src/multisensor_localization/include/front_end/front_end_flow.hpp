#ifndef _FRONT_END_FLOW_H
#define _FRONT_END_FLOW_H

//通用头文件
#include "../../head.hpp"
//前端里程计
#include "./front_end.hpp"
//传感器数据类型
#include "../include/sensor_data/cloud_data.hpp"
#include "../include/sensor_data/gnss_data.hpp"
#include "../include/sensor_data/imu_data.hpp"
//话题订阅
#include "../include/subscriber/cloud_subscriber.hpp"
#include "../include/subscriber/imu_subscriber.hpp"
#include "../include/subscriber/gnss_subscriber.hpp"
#include "../include/tf_listener/tf_listener.hpp"
//话题发布
#include "../include/publisher/odom_publisher.hpp"
#include "../include/publisher/cloud_publisher.hpp"
#include "../include/publisher/origin_publisher.hpp"

namespace multisensor_localization
{

    class FrontEndFlow
    {
    public:
        /*构造函数完成初始化*/
        FrontEndFlow(ros::NodeHandle &nh);
        bool Run();

    private:
        bool ReadData();
        bool Calibration();
        bool InitGnss();
        bool HasData();
        bool ValidData();
        bool UpdateGnssOdom();
        bool UpdateLaserOdom();
        bool PublishData();

    private:
        /*前端里程计*/
        shared_ptr<FrontEnd> front_end_ptr_;
        /*话题订阅*/
        shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        shared_ptr<ImuSubscriber> imu_sub_ptr_;
        shared_ptr<GnssSubscriber> gnss_sub_ptr_;
        shared_ptr<TfListener> lidar_to_imu_ptr_;
        /*话题发布*/
        shared_ptr<CloudPublisher> current_scan_pub_ptr_;
        shared_ptr<CloudPublisher> local_map_pub_ptr_;
        shared_ptr<CloudPublisher> global_map_pub_ptr_;
        shared_ptr<OriginPublisher> origin_pub_ptr_;
        shared_ptr<OdomPublisher> gnss_odom_pub_ptr_;
        shared_ptr<OdomPublisher> laser_odom_pub_ptr_;
        /*地图指针*/
        CloudData::CLOUD_PTR current_scan_ptr_;
        CloudData::CLOUD_PTR local_map_ptr_;
        CloudData::CLOUD_PTR global_map_ptr_;
        /*坐标变换*/
          Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();
        /*里程计*/
        Eigen::Matrix4f gnss_odom_= Eigen::Matrix4f::Identity();
        Eigen::Matrix4f laser_odom_= Eigen::Matrix4f::Identity();
        /*队列中传感器数据*/
        deque<CloudData> cloud_data_buff_;
        deque<ImuData> imu_data_buff_;
        deque<GnssData> gnss_data_buff_;
        /*当前传感器数据*/
        CloudData current_cloud_data_;
        ImuData current_imu_data_;
        GnssData current_gnss_data_;
    };

} // namespace multisensor_localization

#endif