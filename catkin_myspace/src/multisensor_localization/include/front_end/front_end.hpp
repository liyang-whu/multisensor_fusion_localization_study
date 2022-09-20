#ifndef _FRONT_END_H
#define _FRONT_END_H

#include "../../head.hpp"
//传感器数据类型
#include "../include/sensor_data/cloud_data.hpp"
//前端配准接口类
#include "../include/models/registration/registration_interface.hpp"
//ndt接口
#include "../include/models/registration/ndt_registration.hpp"
namespace multisensor_localization
{
    class FrontEnd
    {

    public:
        struct Frame
        {
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            CloudData cloud_data;
        };

    public:
        FrontEnd();
        bool InitWithConfig();
        bool InitDataPath(const YAML::Node &config_node);
        bool InitRegistration(shared_ptr<RegistrationInterface> &registeration_ptr, const YAML::Node  &config_node);

    private:
        string data_path_ = "";

        shared_ptr<RegistrationInterface> registration_ptr_;

        CloudData::CLOUD_PTR local_map_ptr_;
        CloudData::CLOUD_PTR global_map_ptr_;
        CloudData::CLOUD_PTR result_map_ptr_;

    

    
    };
}
#endif
