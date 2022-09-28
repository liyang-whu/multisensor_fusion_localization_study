/*
 * @Description: front_end_flow 前端里程计算任务管理器
 * @Author: robotics 港
 * @Date: 2022-9-26
 * @Note: Modifiled from Ren Qian (github.com/Little-Potato-1990/localization_in_auto_driving)
 */

#include "../../include/front_end/front_end_flow.hpp"

namespace multisensor_localization
{
    /**
     * @brief 前端任务管理初始化
     * 初始化订阅器、发布器、前端里程计
     * @note 调用了FrontEnd()构造
     * @todo 话题配置参数化
     **/
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
        current_scan_ptr_.reset(new CloudData::CLOUD());
        local_map_ptr_.reset(new CloudData::CLOUD());
        global_map_ptr_.reset(new CloudData::CLOUD());
    }

    /**
     * @brief 前端任务管理调度执行
     * @note
     * @todo
     **/
    bool FrontEndFlow::Run()
    {
        /*数据读取*/
        if (!ReadData())
            return false;

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
            if (UpdateLaserOdom())
            {

                LOG(INFO) << endl
                          << fontColorYellow << "激光里程计" << fontColorReset << endl
                          << fontColorBlue << laser_odom_ << fontColorReset << endl
                          << endl;
                /*发布可视化信息*/
                PublishData();
                /*保存轨迹进行evo评估*/
                SaveTrajectory();
            }
        }

        return true;
    }

    /**
     * @brief  从缓冲区读取到传感器数据队列中
     * 点云 imu gnss
     * @note
     * @todo
     **/
    bool FrontEndFlow::ReadData()
    {
        cloud_sub_ptr_->ParseData(cloud_data_buff_);
        imu_sub_ptr_->ParseData(imu_data_buff_);
        gnss_sub_ptr_->ParseData(gnss_data_buff_);

        return true;
    }

    /**
        @brief 多传感器标定
        @note
        @todo yaml读参已标定数据或者支持在线标定模块
    **/
    bool FrontEndFlow::Calibration()
    {
        static bool calibration_inited = false;
        if (calibration_inited == false)
        {
            /*lidar imu标定位*/
            lidar_to_imu_ << 0.999998, 0.000755307, -0.00203583, -0.808676,
                -0.000785403, 0.99989, -0.014823, 0.319556,
                0.00202441, 0.0148245, 0.999888, -0.799723,
                0, 0, 0, 1;
            calibration_inited = true;

            LOG(INFO) << endl
                      << fontColorYellow << "lidar-imu标定完成" << fontColorReset << endl
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
        static bool gnss_inited = false;
        if (!gnss_inited && gnss_data_buff_.size() > 0)
        {
            /*以gnss队列初数据作为定位原点*/
            GnssData gnss_data_origin = gnss_data_buff_.front();
            gnss_data_origin.InitOriginPosition();
            gnss_inited = true;

            LOG(INFO) << endl
                      << fontColorYellow << "gnss初始化完成" << fontColorReset << endl
                      << fontColorBlue << "经度" << gnss_data_origin.longtitude_ << fontColorReset << endl
                      << fontColorBlue << "纬度" << gnss_data_origin.latitude_ << fontColorReset << endl
                      << fontColorBlue << "海拔" << gnss_data_origin.altitude_ << fontColorReset << endl
                      << endl;
            DisplayProgress("【part2 传感器初始化完成】");

            origin_pub_ptr_->Publish(gnss_data_origin);
        }
        return gnss_inited;
    }

    /**
        @brief 检查传感器队列中是否有数据
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
        @note 时间戳对齐
        @todo 多传感器时间戳插值
    **/
    bool FrontEndFlow::ValidData()
    {
        /*从传感器队列取出数据做当前数据*/
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
        /*对齐已取出的情况下，丢弃旧数据*/
        cloud_data_buff_.pop_front();
        imu_data_buff_.pop_front();
        gnss_data_buff_.pop_front();

        return true;
    }

    /**
    @brief 更新gnss里程计
    @note ?是否会有false的情况
    @todo
    **/
    bool FrontEndFlow::UpdateGnssOdom()
    {
        gnss_odom_ = Eigen::Matrix4f::Identity();
        /*利用地理日志库更新东北天坐标系三轴位移*/
        current_gnss_data_.UpdateXYZ();
        gnss_odom_(0, 3) = current_gnss_data_.local_E_;
        gnss_odom_(1, 3) = current_gnss_data_.local_N_;
        gnss_odom_(2, 3) = current_gnss_data_.local_U_;
        /*imu信息补充三轴旋转*/
        gnss_odom_.block<3, 3>(0, 0) = current_imu_data_.OrientationToRotation();
        gnss_odom_ *= lidar_to_imu_;

        return true;
    }

    /**
    @brief 更新激光里程计
    @note ?是否会有false的情况
    @todo
    **/
    bool FrontEndFlow::UpdateLaserOdom()
    {

        /*利用gnss初始位姿初始化激光里程计位姿*/
        static bool front_end_pose_inited = false;
        if (!front_end_pose_inited)
        {
            front_end_pose_inited = true;
            front_end_ptr_->SetInitPose(gnss_odom_);

            LOG(INFO) << endl
                      << fontColorYellow << "激光里程计初始化位姿" << fontColorReset << endl
                      << fontColorBlue << gnss_odom_ << fontColorReset << endl
                      << endl;
            return front_end_ptr_->Update(current_cloud_data_, laser_odom_);
        }
        /*更新激光里程计*/
        laser_odom_ = Eigen::Matrix4f::Identity();
        return front_end_ptr_->Update(current_cloud_data_, laser_odom_);
    }

    /**
    @brief 发布可视化信息
    @note
    @todo
    **/
    bool FrontEndFlow::PublishData()
    {
        /*里程计信息发布*/
        gnss_odom_pub_ptr_->Publish(gnss_odom_);
        laser_odom_pub_ptr_->Publish(laser_odom_);
        /*发布当前扫描点云*/
        front_end_ptr_->GetCurrentScan(current_scan_ptr_);
        current_scan_pub_ptr_->Publish(current_scan_ptr_);
        /*发布局部地图(当有更新时)*/
        if (front_end_ptr_->GetNewLocalMap(local_map_ptr_))
        {
            local_map_pub_ptr_->Publish(local_map_ptr_);
        }

        return true;
    }

    /**
     @brief 保存地图
     @note
     @todo
     **/
    bool FrontEndFlow::SaveMap()
    {
        return front_end_ptr_->SaveMap();
    }

    /**
     @brief 发布全局地图
     @note
     @todo
    **/
    bool FrontEndFlow::PublishGlobalMap()
    {
        if (front_end_ptr_->GetNewGlobalMap(global_map_ptr_))
        {
            global_map_pub_ptr_->Publish(global_map_ptr_);
            global_map_ptr_.reset(new CloudData::CLOUD());
        }
        return true;
    }

    /**
     @brief 保存轨迹
    @note evo评估对比
    @todo
    **/
    bool FrontEndFlow::SaveTrajectory()
    {
        /*创建文件夹及文件*/
        static ofstream ground_truth_ofs, laser_odom_ofs;
        static bool has_file_created = false;
        if (!has_file_created)
        {
            string work_sapce_path = ros::package::getPath("multisensor_localization");
            if (!FileManager::CreateDirectory(work_sapce_path + "/data/trajectory"))
                return false;
            if (!FileManager::CreateFile(ground_truth_ofs, work_sapce_path + "/data/trajectory/ground_truth.txt"))
                return false;
            if (!FileManager::CreateFile(laser_odom_ofs, work_sapce_path + "/data/trajectory/laser_odom.txt"))
                return false;
            has_file_created = true;
        }
        /*写入数据*/
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                ground_truth_ofs << gnss_odom_(i, j);
                laser_odom_ofs << laser_odom_(i, j);
                if (i == 2 && j == 3)
                {
                    ground_truth_ofs << endl;
                    laser_odom_ofs << endl;
                }
                else
                {
                    ground_truth_ofs << " ";
                    laser_odom_ofs << " ";
                }
            }
        }
        return true;
    }

}