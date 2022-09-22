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
}