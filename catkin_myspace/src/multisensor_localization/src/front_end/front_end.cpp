/*
 * @Description: front_end_flow 前端里程计算法
 * @Author: robotics 港
 * @Date: 2022-9-26
 * @Note: Modifiled from Ren Qian (github.com/Little-Potato-1990/localization_in_auto_driving)
 */

#include "../../include/front_end/front_end.hpp"

namespace multisensor_localization
{
    /**
     * @brief 前端算法初始化
     * 调用参数配置函数
     * @note ?result_map_ptr_
     * @todo
     **/
    FrontEnd::FrontEnd()
        : local_map_ptr_(new CloudData::CLOUD),
          global_map_ptr_(new CloudData::CLOUD),
          result_map_ptr_(new CloudData::CLOUD)
    {
        /*参数配置*/
        InitWithConfig();
    }

    /**
     * @brief 参数配置:
     * 配置点云存放路径、点云匹滤波参数、点云匹配参数方式、局部地图参数
     * @note yaml node可递归
     * @todo
     **/
    bool FrontEnd::InitWithConfig()
    {
        /*加载yaml参数文件*/
        string config_file_path = ros::package::getPath("multisensor_localization") + "/config/front_end/config.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);
        /*配置数据存放路径*/
        InitDataPath(config_node);
        /*配置点云滤波方式*/
        InitFilter("local_map", local_map_filter_ptr_, config_node);
        InitFilter("frame", frame_filter_ptr_, config_node);
        InitFilter("display", display_filter_ptr_, config_node);
        /*配置点云匹配方式*/
        InitRegistration(registration_ptr_, config_node);
        /*配置局部地图参数*/
        InitLocalMap(config_node);

        DisplayProgress("【Part1 参数配置完成】");

        return true;
    }

    /**
     * @brief 配置数据存放路径
     * @note LOG会记录并打印文件存储位置
     * @todo
     **/
    bool FrontEnd::InitDataPath(const YAML::Node &config_node)
    {
        /*读取 "点云数据存放地址" 分别处理相对路径和绝对路径*/
        data_path_ = config_node["data_path"].as<string>();
        if (data_path_ == "./")
        {
            data_path_ = ros::package::getPath("multisensor_localization") + "/data";
        }
        else
        {
            data_path_ += "/data";
        }
        /*是否已存在该文件夹 存在则删除*/
        if (boost::filesystem::is_directory(data_path_))
        {
            boost::filesystem::remove_all(data_path_);
        }
        /*尝试创建新文件夹*/
        try
        {
            boost::filesystem::create_directory(data_path_);
        }
        catch (const boost::filesystem::filesystem_error &e)
        {
            LOG(INFO) << endl
                      << fontColorRedBold << "数据存放文件夹路径非法" << endl
                      << fontColorReset << endl;
            ROS_BREAK();
        }
        /*检查slam_data文件夹是否创建成功*/
        if (boost::filesystem::is_directory(data_path_))
        {
            LOG(INFO) << endl
                      << fontColorYellow << "slam_data文件夹创建成功" << fontColorReset << endl
                      << fontColorBlue << data_path_ << fontColorReset << endl
                      << endl;
        }
        else
        {
            LOG(INFO) << endl
                      << fontColorRedBold << "slam_data文件夹创建失败" << endl
                      << fontColorReset << endl;
            ROS_BREAK();
        }
        /*检查data/key_frame文件夹是否创建成功*/
        string key_frame_path = data_path_ + "/key_frame";
        boost::filesystem::create_directory(key_frame_path);
        if (boost::filesystem::is_directory(key_frame_path))
        {
            LOG(INFO) << endl
                      << fontColorYellow << "slam_data/key_frame 文件夹创建成功" << fontColorReset << endl
                      << fontColorBlue << key_frame_path << fontColorReset << endl
                      << endl;
        }
        else
        {
            LOG(INFO) << endl
                      << fontColorRedBold << "slam_data/key_frame 文件夹创建失败" << endl
                      << fontColorReset << endl;
            ROS_BREAK();
        }

        return true;
    }

    /**
     * @brief 配置点云滤波方法
     * @note
     * @todo
     **/
    bool FrontEnd::InitFilter(string filter_user,
                              shared_ptr<CloudFilterInterface> &filter_ptr,
                              const YAML::Node &config_node)
    {
        /*读参"滤波方法"*/
        string filter_method = config_node[filter_user + "_filter"].as<string>();
        /*检查滤波方式*/
        if (filter_method == "voxel_filter")
        {
            LOG(INFO) << endl
                      << fontColorYellow << filter_user << "滤波方式" << fontColorReset << endl
                      << fontColorBlue << filter_method << fontColorReset << endl
                      << endl;
            filter_ptr = make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
        }
        else
        {
            LOG(INFO) << endl
                      << fontColorRedBold << "无对应的滤波方法" << endl
                      << fontColorReset << endl;
            ROS_BREAK();
        }

        return true;
    }

    /**
     * @brief 配置点云配准方式
     * @note
     * @todo
     **/
    bool FrontEnd::InitRegistration(shared_ptr<RegistrationInterface> &registeration_ptr, const YAML::Node &config_node)
    {
        /*读参"点云匹配方法"*/
        string registration_method = config_node["registration_method"].as<string>();
        /*检查点云匹配方法*/
        if (registration_method == "NDT")
        {
            LOG(INFO) << endl
                      << fontColorYellow << "点云匹配方式" << fontColorReset << endl
                      << fontColorBlue << registration_method << fontColorReset << endl
                      << endl;
            registeration_ptr = make_shared<NdtRegistration>(config_node[registration_method]);
        }
        else
        {
            LOG(INFO) << endl
                      << fontColorRedBold << "无对应的点云匹配方法" << endl
                      << fontColorReset << endl;
            ROS_BREAK();
        }
        return true;
    }

    /**
     * @brief 配置局部地图数据
     * @note
     * @todo
     **/
    bool FrontEnd::InitLocalMap(const YAML::Node &config_node)
    {
        key_frame_distance_ = config_node["key_frame_distance"].as<float>();
        local_frame_num_ = config_node["local_frame_num"].as<float>();

        LOG(INFO) << endl
                  << fontColorYellow << "局部地图参数 " << fontColorReset << endl
                  << fontColorBlue << "关键帧提取距离  " << key_frame_distance_ << fontColorReset << endl
                  << fontColorBlue << "关键帧队列数量  " << local_frame_num_ << fontColorReset << endl
                  << endl;

        return true;
    }

    /**
    @brief 设置激光里程计位姿原点
    @note
    @todo
**/
    bool FrontEnd::SetInitPose(const Eigen::Matrix4f &init_pose)
    {
        init_pose_ = init_pose;
        return true;
    }

    /**
    @brief 里程计更新
    @note
    @todo 位姿预测融合imu信息
**/
    bool FrontEnd::Update(const CloudData &cloud_data, Eigen::Matrix4f &cloud_pose)
    {
        /*拷贝时间戳*/
        current_frame_.cloud_data.time_stamp_ = cloud_data.time_stamp_;
        /*去除无效点云*/
        vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr_,
                                     *current_frame_.cloud_data.cloud_ptr_,
                                     indices);
        /*点云滤波 降采样*/
        CloudData::CLOUD_PTR filtered_cloud_ptr_(new CloudData::CLOUD);
        frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr_,
                                  filtered_cloud_ptr_);

        /*位姿预测所需的递推变量*/
        static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity(); // t-1到t时刻的步长增量
        static Eigen::Matrix4f last_pose = init_pose_;                  // t-1时刻的位姿
        static Eigen::Matrix4f predict_pose = init_pose_;               //预测的t+1时刻的位姿
        static Eigen::Matrix4f last_key_frame_pose = init_pose_;        //上一关键帧的位姿

        /*当前局部地图为空时 初始化(使用初始位姿)*/
        if (local_map_frames_.size() == 0)
        {
            current_frame_.pose = init_pose_;
            UpdateNewFrame(current_frame_);
            return true;
        }

        /*当前局部地图不为空时 (匹配时用滤波后点云)*/
        registration_ptr_->ScanMatch(filtered_cloud_ptr_,  //滤波后的点云
                                     predict_pose,         //预测位姿
                                     result_map_ptr_,      //转换到laser_odom下的点云
                                     current_frame_.pose); //当前位姿势
        /*通过传值给引用入参传递位姿势结果*/
        cloud_pose = current_frame_.pose;

        /*更新两帧相对运动  为点云匹配计算出相对准确的初值以加快匹配速度*/
        step_pose = last_pose.inverse() * current_frame_.pose;
        predict_pose = current_frame_.pose * step_pose;
        last_pose = current_frame_.pose;

        /*依据欧式距离判断是否需要关键帧 (更新时用滤波前的点云)*/
        float distance_forward = fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) +
                                 fabs(last_key_frame_pose(1, 3) - current_frame_.pose(1, 3)) +
                                 fabs(last_key_frame_pose(2, 3) - current_frame_.pose(2, 3));

        if (distance_forward > key_frame_distance_)
        {
            UpdateNewFrame(current_frame_);
            last_key_frame_pose = current_frame_.pose;
        }

        // LOG(INFO) << endl
        //           << fontColorGreen << ">> >> >> >> >> debug point >> >> >> >> >>" << endl
        //           << fontColorYellow << "激光里程计" << fontColorReset << endl
        //           << fontColorBlue << cloud_pose << fontColorReset << endl
        //           << fontColorGreen << "<< << << << <<  debug point << << << << <<" << endl
        //           << endl;

        return true;
    }

    /**
    @brief 更新关键帧
    @note 此时new_key_frame已含有位姿数据
    @todo
**/
    bool FrontEnd::UpdateNewFrame(const Frame &new_key_frame)
    {
        /*保存关键帧到硬盘中 节省内存*/
        string file_path = data_path_ + "/key_frame/key_frame_" +\
         to_string(global_map_frames_.size()) + ".pcd";
        pcl::io::savePCDFileBinary(file_path, *new_key_frame.cloud_data.cloud_ptr_);

        /*点云深拷贝*/
        Frame key_frame = new_key_frame;//这样拷贝只能拷贝出指针 为浅拷贝
        key_frame.cloud_data.cloud_ptr_.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr_));

        /*更新局部地图,维护局部地图帧数不变*/
        local_map_frames_.push_back(key_frame);
        while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_))
        {
            local_map_frames_.pop_front();
        }

        /*根据计算出的位姿转换原点处的点云到激光里程计下*/
           CloudData::CLOUD_PTR fransformed_cloud_ptr(new CloudData::CLOUD());
        local_map_ptr_.reset(new CloudData::CLOUD());
        for (size_t i = 0; i < local_map_frames_.size(); i++)
        {
            pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr_,
                                     *fransformed_cloud_ptr,
                                     local_map_frames_.at(i).pose);
            *local_map_ptr_ += *fransformed_cloud_ptr;
        }
        has_new_local_map_ = true;

        /*设置DNT匹配的目标点云 帧数太少不滤波*/
        if (local_map_frames_.size() < 10)
        {
            registration_ptr_->SetTarget(local_map_ptr_);
        }
        else
        {
            CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
            local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
            registration_ptr_->SetTarget(filtered_local_map_ptr);
        }

        /*关键帧已保存至容器,释放当前关键帧但保留位姿*/
        key_frame.cloud_data.cloud_ptr_.reset(new CloudData::CLOUD());
        /*利用global_map_frames_.size记序号和每帧位姿*/
        global_map_frames_.push_back(key_frame);

        return true;
    }

    /**
    @brief 滤波当前扫描帧
    @note
    @todo
**/
    bool FrontEnd::GetCurrentScan(CloudData::CLOUD_PTR &current_map_ptr)
    {
        display_filter_ptr_->Filter(result_map_ptr_, current_map_ptr);
        return true;
    }

    /**
    @brief 滤波局部地图
    @note
    @todo
**/
    bool FrontEnd::GetNewLocalMap(CloudData::CLOUD_PTR &local_map_ptr)
    {
        if (has_new_local_map_ == true)
        {
            display_filter_ptr_->Filter(local_map_ptr_, local_map_ptr);
        }
        return true;
    }



    bool FrontEnd::SaveMap()
    {
        global_map_ptr_.reset(new CloudData::CLOUD());
        string key_frame_path = "";
        CloudData::CLOUD_PTR key_frame_cloud_ptr(new CloudData::CLOUD());
        CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

        /*硬盘中依次读取关键帧序列并集合至全局地图*/
        for (unsigned int i = 0; i < global_map_frames_.size(); i++)
        {
            key_frame_path = data_path_ +\
             "/key_frame/key_frame_" + to_string(i) + ".pcd";
            pcl::io::loadPCDFile(key_frame_path, *key_frame_cloud_ptr);
            pcl::transformPointCloud(*key_frame_cloud_ptr,
                                     *transformed_cloud_ptr,
                                     global_map_frames_.at(i).pose);

            *global_map_ptr_ += *transformed_cloud_ptr;
        }
         /*保存全局地图至硬盘*/
        string map_file_path = data_path_ + "/map.pcd";
        pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr_);
        has_new_global_map_ = true;

        return true;
    }

    bool FrontEnd::GetNewGlobalMap(CloudData::CLOUD_PTR &global_map_ptr)
    {
        if (has_new_global_map_ == true)
        {
            has_new_global_map_ = false;
            display_filter_ptr_->Filter(global_map_ptr, global_map_ptr);
            global_map_ptr_.reset(new CloudData::CLOUD());
            return true;
        }
        return false;
    }

 
}