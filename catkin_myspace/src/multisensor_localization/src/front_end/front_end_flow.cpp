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
        front_end_ptr_ = make_shared<FrontEnd>();
        /*重置地图指针*/
        local_map_ptr_.reset(new CloudData::CLOUD());
        global_map_ptr_.reset(new CloudData::CLOUD());
        current_scan_ptr_.reset(new CloudData::CLOUD());
    }

    bool FrontEndFlow::Run()
    {

        /*数据读取*/
        ReadData();
        /*传感器标定*/
        if (!Calibration())
            return false;
        /*gnss初始化*/
        if (!InitGnss())
            return false;

        while (HasData())
        {
            /*有效数据进行处理 无效数据直接跳过*/
            if (!ValidData())
                continue;

            /*更新gnss imu里程计*/
            UpdateGnssOdom();
            /*更新laser里程计*/
            UpdateLaserOdom();
            /*发布可视化信息*/
            PublishData();
        }

        return true;
    }

    /**
          @brief 从缓冲区读取数据到队列
         */
    bool FrontEndFlow::ReadData()
    {
        cloud_sub_ptr_->ParseData(cloud_data_buff_);
        imu_sub_ptr_->ParseData(imu_data_buff_);
        gnss_sub_ptr_->ParseData(gnss_data_buff_);
    }

    /**
        @brief 多传感器标定
        @note
        @todo
    **/
    bool FrontEndFlow::Calibration()
    {
        static bool calibration_inited = false;
        if (calibration_inited == false)
        {
            lidar_to_imu_ << 0.999998, 0.000755307, -0.00203583, -0.808676,
                -0.000785403, 0.99989, -0.014823, 0.319556,
                0.00202441, 0.0148245, 0.999888, -0.799723,
                0, 0, 0, 1;
            calibration_inited = true;

            LOG(INFO) << endl
                      << fontColorYellow << "lidar imu标定完成" << fontColorReset << endl
                      << fontColorBlue << lidar_to_imu_ << fontColorReset << endl
                      << endl;
        }
        return calibration_inited;
    }

    /**
        @brief 初始化gnss
        @note
        @todo
    **/
    bool FrontEndFlow::InitGnss()
    {
        static bool gnss_init = false;
        if (!gnss_init && gnss_data_buff_.size() > 0)
        {
            GnssData gnss_data_origin = gnss_data_buff_.front();
            gnss_data_origin.InitOriginPosition();
            gnss_init = true;
            LOG(INFO) << endl
                      << fontColorYellow << "gnss初始化完成" << fontColorReset << endl
                      << fontColorBlue << "经度" << gnss_data_origin.longtitude_ << fontColorReset << endl
                      << fontColorBlue << "纬度" << gnss_data_origin.latitude_ << fontColorReset << endl
                      << fontColorBlue << "海拔" << gnss_data_origin.altitude_ << fontColorReset << endl
                      << endl;
            origin_pub_ptr_->Publish(gnss_data_origin);
        }
        return gnss_init;
    }

    /**
        @brief 检查数据队列中是否有数据
        @note
        @todo
    **/
    bool FrontEndFlow::HasData()
    {
        if (cloud_data_buff_.size() == 0)
            return false;
        if (imu_data_buff_.size() == 0)
            return false;
        if (gnss_data_buff_.size() == 0)
            return false;

        return true;
    }

    /**
        @brief 提取有效数据
        @note
        @todo
    **/
    bool FrontEndFlow::ValidData()
    {
        current_cloud_data_ = cloud_data_buff_.front();
        current_imu_data_ = imu_data_buff_.front();
        current_gnss_data_ = gnss_data_buff_.front();

        double d_time = current_cloud_data_.time_stamp_ - current_imu_data_.time_stamp_;

        /*点云数据超前 惯导数据滞后*/
        if (d_time > 0.05)
        {
            imu_data_buff_.pop_front();
            gnss_data_buff_.pop_front();
            return false;
        }

        /*点云数据滞后  惯导数据超前*/
        if (d_time < -0.05)
        {
            cloud_data_buff_.pop_front();
            return false;
        }

        cloud_data_buff_.pop_front();
        imu_data_buff_.pop_front();
        gnss_data_buff_.pop_front();
        return true;
    }

    bool FrontEndFlow::UpdateGnssOdom()
    {
        gnss_odom_ = Eigen::Matrix4f::Identity();
        current_gnss_data_.UpdateXYZ();
        gnss_odom_(0, 3) = current_gnss_data_.local_E_;
        gnss_odom_(1, 3) = current_gnss_data_.local_N_;
        gnss_odom_(2, 3) = current_gnss_data_.local_U_;

        gnss_odom_.block<3, 3>(0, 0) = current_imu_data_.OrientationToRotation();
        gnss_odom_ *= lidar_to_imu_;
    }

    bool FrontEndFlow::UpdateLaserOdom()
    {

        /*利用gnss进行初始化*/
        static bool front_end_pose_inited = false;
        if (!front_end_pose_inited)
        {
            front_end_pose_inited = true;
            front_end_ptr_->SetInitPose(gnss_odom_);

            laser_odom_ = gnss_odom_;

            LOG(INFO) << endl
                      << fontColorYellow << "激光里程计初始化位姿" << fontColorReset << endl
                      << fontColorBlue << laser_odom_ << fontColorReset << endl
                      << endl;

            return true;
        }
        /*更新激光里程计*/
        laser_odom_ = Eigen::Matrix4f::Identity();
        front_end_ptr_->Update(current_cloud_data_, laser_odom_);
        return true;
    }

    bool FrontEndFlow::PublishData()
    {
        gnss_odom_pub_ptr_->Publish(gnss_odom_);
        laser_odom_pub_ptr_->Publish(laser_odom_);

        if(front_end_ptr_->GetNewLocalMap(local_map_ptr_))
        {
            local_map_pub_ptr_->Publish(local_map_ptr_);
        }

        front_end_ptr_->GetCurrentScan(current_scan_ptr_);
        current_scan_pub_ptr_->Publish(current_scan_ptr_);

        return true;
    }


    
}