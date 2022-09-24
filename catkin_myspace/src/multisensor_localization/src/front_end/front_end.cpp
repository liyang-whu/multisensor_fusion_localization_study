#include "../../include/front_end/front_end.hpp"

namespace multisensor_localization
{

    /**
     @brief 前端里程计算初始化
    */
    FrontEnd::FrontEnd()
        : local_map_ptr_(new CloudData::CLOUD),
          global_map_ptr_(new CloudData::CLOUD),
          result_map_ptr_(new CloudData::CLOUD)
    {

        /*参数配置*/
        InitWithConfig();
    }

    bool FrontEnd::Update(const CloudData &cloud_data, Eigen::Matrix4f &cloud_pose)
    {
        current_frame_.cloud_data.time_stamp_ = cloud_data.time_stamp_;
        /*去除无效点云*/
        vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr_, *current_frame_.cloud_data.cloud_ptr_, indices);
        /*点云滤波*/
        CloudData::CLOUD_PTR filtered_cloud_ptr_(new CloudData::CLOUD);
        frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr_, filtered_cloud_ptr_);

        // LOG(INFO) << endl
        //           << fontColorGreen << ">> >> >> >> >> debug point >> >> >> >> >>" << endl
        //           << fontColorYellow << "原始点云规模" << fontColorReset << endl
        //           << fontColorBlue << (*cloud_data.cloud_ptr_).size() << fontColorReset << endl
        //           << fontColorYellow << "去无效点后点云规模" << fontColorReset << endl
        //           << fontColorBlue << (*current_frame_.cloud_data.cloud_ptr_).size() << fontColorReset << endl
        //           << fontColorYellow << "降采样后点云规模" << fontColorReset << endl
        //           << fontColorBlue << (*filtered_cloud_ptr_).size() << fontColorReset << endl
        //           << fontColorGreen << "<< << << << <<  debug point << << << << <<" << endl
        //           << endl;

        static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
        static Eigen::Matrix4f last_pose = init_pose_;
        static Eigen::Matrix4f predict_pose = init_pose_;
        static Eigen::Matrix4f last_key_frame_pose = init_pose_;

        /*当前局部地图为空时 初始化(使用初始位姿)*/
        if (local_map_frames_.size() == 0)
        {
            current_frame_.pose = init_pose_;
            UpdateNewFrame(current_frame_);
            return true;
        }

        /*当前局部地图不为空时 (匹配时用滤波后点云)*/
        registration_ptr_->ScanMatch(filtered_cloud_ptr_,
                                     predict_pose,
                                     result_map_ptr_, //投影到laser_odom下的点云可显示
                                     current_frame_.pose);
        cloud_pose = current_frame_.pose;

        /*更新两帧相对运动*/
        step_pose = last_pose.inverse() * current_frame_.pose;
        predict_pose = current_frame_.pose * step_pose;
        last_pose = current_frame_.pose;

        /*依据欧式距离判断是否需要关键帧 (更新时用滤波前的点云)*/
        float distance_forward = fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) +
                                 fabs(last_key_frame_pose(1, 3) - current_frame_.pose(1, 3)) +
                                 fabs(last_key_frame_pose(2, 3) - current_frame_.pose(2, 3));

        if (distance_forward > key_frame_distance)
        {
            UpdateNewFrame(current_frame_);
            last_key_frame_pose = current_frame_.pose;
        }

        LOG(INFO) << endl
                  << fontColorGreen << ">> >> >> >> >> debug point >> >> >> >> >>" << endl
                  << fontColorYellow << "激光里程计" << fontColorReset << endl
                  << fontColorBlue << cloud_pose << fontColorReset << endl
                  << fontColorGreen << "<< << << << <<  debug point << << << << <<" << endl
                  << endl;

        return true;
    }

    /**
      @brief 前端里程计参数配置
     */
    bool FrontEnd::InitWithConfig()
    {
        /*加载yaml参数文件*/
        string config_file_path = ros::package::getPath("multisensor_localization") + "/config/front_end/config.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);
        /*设置点云存放路径*/
        InitDataPath(config_node);
        /*设置点云匹配方式*/
        InitRegistration(registration_ptr_, config_node);
        /*设置点云滤波方式*/
        InitFilter("local_map", local_map_filter_ptr_, config_node);
        InitFilter("frame", frame_filter_ptr_, config_node);
        InitFilter("display", display_filter_ptr_, config_node);

        return true;
    }

    /**
        @brief 创建点云数据文件夹
       */
    bool FrontEnd::InitDataPath(const YAML::Node &config_node)
    {
        /*读取到参数:点云数据存放地址*/
        data_path_ = config_node["data_path"].as<string>() + "/slam_data";

        /*删除旧文件并创建新文件夹*/
        boost::filesystem::remove_all(data_path_);
        boost::filesystem::create_directory(data_path_);

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
        /*检查slam_data/key_frame文件夹是否创建成功*/
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

    bool FrontEnd::InitRegistration(shared_ptr<RegistrationInterface> &registeration_ptr, const YAML::Node &config_node)
    {
        /*匹配方法读参*/
        string registration_method = config_node["registration_method"].as<string>();

        if (registration_method == "NDT")
        {
            registeration_ptr = make_shared<NdtRegistration>(config_node[registration_method]);
            LOG(INFO) << endl
                      << fontColorYellow << "点云匹配方式" << fontColorReset << endl
                      << fontColorBlue << registration_method << fontColorReset << endl
                      << endl;
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

    bool FrontEnd::InitFilter(string filter_user, shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node)
    {
        /*读参滤波方法*/
        string filter_method = config_node[filter_user + "_filter"].as<string>();

        LOG(INFO) << endl
                  << fontColorYellow << filter_user << "滤波方式" << fontColorReset << endl
                  << fontColorBlue << filter_method << fontColorReset << endl
                  << endl;

        /*设置滤波方法*/
        if (filter_method == "voxel_filter")
        {
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

    bool FrontEnd::SetInitPose(const Eigen::Matrix4f &init_pose)
    {
        init_pose_ = init_pose;
    }

    /**
     * @brief 更新队列里的关键帧
     * @note 关键输入的帧保证已知自身点云、位姿
     * */
    bool FrontEnd::UpdateNewFrame(const Frame &new_key_frame)
    {
        /*保存关键帧到硬盘中 节省内存*/
        string file_path = data_path_ + "/key_frame/key_frame_" + to_string(global_map_frames_.size()) + ".pcd";
        pcl::io::savePCDFileBinary(file_path, *new_key_frame.cloud_data.cloud_ptr_);

        /*点云深拷贝*/
        Frame key_frame = new_key_frame;
        key_frame.cloud_data.cloud_ptr_.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr_));

        CloudData::CLOUD_PTR fransformed_cloud_ptr(new CloudData::CLOUD());

        /*更新局部地图,维护局部地图规模不变*/
        local_map_frames_.push_back(key_frame);
        while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_))
        {
            local_map_frames_.pop_front();
        }

        /*转换局部地图的坐标系到当前坐标系下*/
        local_map_ptr_.reset(new CloudData::CLOUD());
        for (size_t i = 0; i < local_map_frames_.size(); i++)
        {
            pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr_,
                                     *fransformed_cloud_ptr,
                                     local_map_frames_.at(i).pose);
            *local_map_ptr_ += *fransformed_cloud_ptr;
        }
        has_new_local_map_ = true;

        /*更新DNT匹配的目标点云*/
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

        /*释放关键帧*/
        key_frame.cloud_data.cloud_ptr_.reset(new CloudData::CLOUD());
        /*利用global_map_frames_.size记序号*/
        global_map_frames_.push_back(key_frame);
    }

    /**
     * @brief 更新队列里的关键帧
     * @note 关键输入的帧保证已知自身点云、位姿
     * */
    bool FrontEnd::GetNewLocalMap(CloudData::CLOUD_PTR &local_map_ptr)
    {
        if (has_new_local_map_ == true)
        {
            display_filter_ptr_->Filter(local_map_ptr_, local_map_ptr);
        }

        return true;
    }

    /**
     * @brief
     * @note
     * */
    bool FrontEnd::GetCurrentScan(CloudData::CLOUD_PTR &current_map_ptr)
    {
        display_filter_ptr_->Filter(result_map_ptr_, current_map_ptr);
        return true;
    }
}