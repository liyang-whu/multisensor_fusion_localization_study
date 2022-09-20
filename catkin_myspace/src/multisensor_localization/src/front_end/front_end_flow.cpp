#include "../../include/front_end/front_end_flow.hpp"

namespace multisensor_localization
{

    FrontEndFlow::FrontEndFlow(ros::NodeHandle &nh)
    {
        /*传感器信息订阅*/
        cloud_sub_ptr_ = make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 1e6);
        imu_sub_ptr_ = make_shared<ImuSubscriber>(nh, "/kitti/oxts/imu", 1e6);
        gnss_sub_ptr_ = make_shared<GnssSubscriber>(nh, "/kitti/oxts/gps/fix", 1e6);
        lidar_to_imu_ptr_ = make_shared<TfListener>(nh, "velo_link", "imu_link");
        /*可视化信息发布*/
        current_scan_pub_ptr_ = make_shared<CloudPublisher>(nh, "current_scan", 100, "map");
        local_map_pub_ptr_ = make_shared<CloudPublisher>(nh, "local_map", 100, "map");
        global_map_pub_ptr_ = make_shared<CloudPublisher>(nh, "global_map", 100, "map");
        origin_pub_ptr_ = make_shared<OriginPublisher>(nh, "ref_point_wgs84", 100, "map");
        gnss_odom_pub_ptr_ = make_shared<OdomPublisher>(nh, "gnss_odom", "map", "lidar", 100);
        laser_odom_pub_ptr_ = make_shared<OdomPublisher>(nh, "lidar_odom", "map", "lidar", 100);
        /*前端里程计*/
        front_end_ptr_=make_shared<FrontEnd>();
        /*重置地图指针*/
        local_map_ptr_.reset(new CloudData::CLOUD());
        global_map_ptr_.reset(new CloudData::CLOUD());
        current_scan_ptr_.reset(new CloudData::CLOUD());
    }

    bool FrontEndFlow::Run()
    {
        return true;
    }

}