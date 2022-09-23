//通用库文件
#include "../include/front_end/front_end_flow.hpp"

using namespace multisensor_localization;
shared_ptr<FrontEndFlow> front_end_flow_ptr;

int main(int argc, char **argv)
{
    /* glog配置 */
    google::InitGoogleLogging(argv[0]);
    string path = ros::package::getPath("multisensor_localization");
    FLAGS_log_dir = path + "/log";
    FLAGS_alsologtostderr = 1;
    boost::filesystem::remove_all(FLAGS_log_dir);
    boost::filesystem::create_directory(FLAGS_log_dir);
    /*节点配置*/
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ros::Rate rate(100);
    /*前端里程计初始化*/
    front_end_flow_ptr = make_shared<FrontEndFlow>(nh);

    while (ros::ok())
    {
        ros::spinOnce();
        front_end_flow_ptr->Run();
        rate.sleep();
    }
    return 0;
}

// int main(int argc, char **argv)
// {
//         google::InitGoogleLogging(argv[0]);
//     ros::init(argc, argv, "test");
//     ros::NodeHandle nh;
//     /*传感器信息订阅*/
//     shared_ptr<CloudSubscriber> cloud_sub_ptr_ = make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 1e6);
//     shared_ptr<ImuSubscriber> imu_sub_ptr_ = make_shared<ImuSubscriber>(nh, "/kitti/oxts/imu", 1e6);
//     shared_ptr<GnssSubscriber> gnss_sub_ptr_ = make_shared<GnssSubscriber>(nh, "/kitti/oxts/gps/fix", 1e6);
//     shared_ptr<TfListener> lidar_to_imu_ptr_ = make_shared<TfListener>(nh, "velo_link", "imu_link");
//     /*可视化数据发布*/

//     deque<ImuData> imu_data_buff;
//     deque<GnssData> gnss_data_buff;
//     deque<CloudData> cloud_data_buff;
//     Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
//     bool transform_received = false;
//     bool gnss_init = false;

//     ros::Rate rate(100);

//     while (ros::ok())
//     {
//         /*传感器信息回调*/
//         ros::spinOnce();

//         /*从缓冲区读入数据*/
//         cloud_sub_ptr_->ParseData(cloud_data_buff);
//         imu_sub_ptr_->ParseData(imu_data_buff);
//         gnss_sub_ptr_->ParseData(gnss_data_buff);

//         if (!transform_received)
//         {
//             if (lidar_to_imu_ptr_->LookUpData(lidar_to_imu))
//             {
//                 transform_received = true;
//             }
//         }
//         else
//         {
//             while (cloud_data_buff.size() > 0 && imu_data_buff.size() > 0 && gnss_data_buff.size() > 0)
//             {
//                 CloudData cloud_data = cloud_data_buff.front();
//                 ImuData imu_data = imu_data_buff.front();
//                 GnssData gnss_data = gnss_data_buff.front();

//                 double d_time = cloud_data.time_stamp_ - imu_data.time_stamp_;

//                 if (d_time < -0.05)
//                 {
//                     cloud_data_buff.pop_front();
//                 }
//                 else if (d_time > 0.05)
//                 {
//                     // kitti使用的为组合惯导
//                     imu_data_buff.pop_front();
//                     gnss_data_buff.pop_front();
//                 }
//                 else
//                 {
//                     /*大致对齐情况下再继续*/
//                     imu_data_buff.pop_front();
//                     cloud_data_buff.pop_front();
//                     gnss_data_buff.pop_front();

//                     Eigen::Matrix4f odom_matrix;

//                     if (!gnss_init)
//                     {
//                         origin_pub_ptr_->Publish(gnss_data);
//                         gnss_data.InitOriginPosition();
//                         gnss_init = true;
//                     }
//                     gnss_data.UpdateXYZ();
//                     odom_matrix(0, 3) = gnss_data.local_E_;
//                     odom_matrix(1, 3) = gnss_data.local_N_;
//                     odom_matrix(2, 3) = gnss_data.local_U_;
//                     odom_matrix.block<3, 3>(0, 0) = imu_data.OrientationToRotation();
//                     odom_matrix *= lidar_to_imu;
//                   //  cout<<"gps imu 里程计数据为:"<<odom_matrix<<endl;
//                     pcl::transformPointCloud(*cloud_data.cloud_ptr_, *cloud_data.cloud_ptr_, odom_matrix);

//                     cloud_pub_ptr_->Publish(cloud_data.cloud_ptr_);
//                     odom_pub_ptr_->Publish(odom_matrix);
//                 }
//             }
//         }

//         rate.sleep();
//     }

//     return 0;
// }
