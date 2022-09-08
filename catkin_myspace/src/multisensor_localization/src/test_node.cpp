#include "../head.hpp"
//话题订阅
#include "../include/subscriber/cloud_subscriber.hpp"
#include "../include/subscriber/imu_subscriber.hpp"
#include "../include/subscriber/gnss_subscriber.hpp"
//话题发布
#include "../include/publisher/odom_publisher.hpp"
#include "../include/publisher/cloud_publisher.hpp""

using namespace multisensor_localization;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    shared_ptr<CloudSubscriber> cloud_sub_ptr_ = make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 1e6);
    shared_ptr<ImuSubscriber> imu_sub_ptr_ = make_shared<ImuSubscriber>(nh, "/kitti/oxts/imu", 1e6);
    shared_ptr<GnssSubscriber> gnss_sub_ptr_ = make_shared<GnssSubscriber>(nh, "/kitti/oxts/gps/fix", 1e6);
    
    
    while (ros::ok())
    {
        //传感器信息回调
        ros::spinOnce();
        cout << "this is just a demo" << endl;
    }

    return 0;
}