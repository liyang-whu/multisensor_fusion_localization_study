#include "../head.hpp"
#include "../include/subscriber/cloud_subscriber.hpp"
#include "../include/subscriber/imu_subscriber.hpp"

using namespace multisensor_localization;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    /*点云订阅*/
    shared_ptr<CloudSubscriber> cloud_sub_ptr_ = make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 1e6);
    /*imu订阅*/
    shared_ptr<ImuSubscriber> imu_sub_ptr_=make_shared<ImuSubscriber>(nh, "/kitti/oxts/imu",1e6);
    
    while (ros::ok())
    {
        //传感器信息回调
        ros::spinOnce();
        cout << "this is just a demo" << endl;
    }

    return 0;
}