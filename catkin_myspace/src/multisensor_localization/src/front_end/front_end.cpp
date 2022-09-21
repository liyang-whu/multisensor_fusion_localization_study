#include "../../include/front_end/front_end.hpp"

namespace multisensor_localization
{
    FrontEnd::FrontEnd()
        : local_map_ptr_(new CloudData::CLOUD),
          global_map_ptr_(new CloudData::CLOUD),
          result_map_ptr_(new CloudData::CLOUD)
    {

        /*参数配置*/
        InitWithConfig();
    }

    bool FrontEnd::InitWithConfig()
    {
        string config_file_path = ros::package::getPath("multisensor_localization") + "/config/front_end/config.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);
        InitDataPath(config_node);
        InitRegistration(registration_ptr_,config_node);
        
        return true;
    }

    bool FrontEnd::InitDataPath(const YAML::Node &config_node)
    {

        data_path_ = config_node["data_path"].as<string>() + "/slam_data";

        boost::filesystem::remove_all(data_path_);
        boost::filesystem::create_directory(data_path_);
        if (boost::filesystem::is_directory(data_path_))
        {
            LOG(INFO) << "【文件夹 data_path创建成功】" << endl;
            LOG(INFO) << "路径为:" << data_path_ << endl;
        }
        else
        {
            LOG(WARNING) << "【文件夹 data_path创建失败】" << endl;
            ROS_BREAK();
        }

        string key_frame_path = data_path_ + "/key_frame";
        boost::filesystem::create_directory(key_frame_path);
        if (boost::filesystem::is_directory(key_frame_path))
        {
            LOG(INFO) << "【文件夹 key_frame_path创建成功】" << endl;
            LOG(INFO) << "路径为:" << key_frame_path << endl;
        }
        else
        {
            LOG(WARNING) << "【文件夹 key_frame_path创建失败】" << endl;
            ROS_BREAK();
        }
        return true;
    }

    bool FrontEnd::InitRegistration(shared_ptr<RegistrationInterface> &registeration_ptr, const YAML::Node &config_node)
    {
        string registration_method = config_node["registration_method"].as<string>();
        LOG(INFO) << "【点云匹配方式】" << registration_method << endl;

        if (registration_method == "NDT")
        {
            registeration_ptr = make_shared<NdtRegistration>(config_node[registration_method]);
        }
        else
        {
            LOG(ERROR) << "未能找到与 " << registration_method << " 所对应的点云匹配方法" << endl;
            return false;
        }
        return true;
    }
}