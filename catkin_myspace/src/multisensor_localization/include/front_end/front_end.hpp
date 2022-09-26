#ifndef _FRONT_END_H
#define _FRONT_END_H

#include "../../head.hpp"
//传感器数据类型
#include "../include/sensor_data/cloud_data.hpp"
//前端配准父类接口
#include "../include/models/registration/registration_interface.hpp"
// ndt方法
#include "../include/models/registration/ndt_registration.hpp"
//点云滤波父类接口
#include "../include/models/cloud_filter/cloud_filter_interface.hpp"
// voxel_filter方法
#include "../include/models/cloud_filter/voxel_filter.hpp"

namespace multisensor_localization
{
    class FrontEnd
    {

    public:
        /*定义关键帧*/
        struct Frame
        {
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            CloudData cloud_data;
        };

    public:
        /*构造函数 前端里程计初始化*/
        FrontEnd();
        /*yaml参数配置入口*/
        bool InitWithConfig();
        /*位姿更新*/
        bool Update(const CloudData&cloud_data,Eigen::Matrix4f &cloud_pose);
        /*设置初始位姿*/
        bool SetInitPose(const Eigen::Matrix4f &init_pose);
        /*获取当前、局部、全局点云*/
        bool GetCurrentScan(CloudData::CLOUD_PTR& current_map_ptr);
        bool GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr);
        bool GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr);
        bool SaveMap();


        private:       
        /*更新关键帧*/
        bool UpdateNewFrame(const Frame&new_key_frame);
         /*各部分yaml参数配置*/
        bool InitParam(const YAML::Node &config_node);
        bool InitDataPath(const YAML::Node &config_node);
        bool InitRegistration(shared_ptr<RegistrationInterface> &registeration_ptr, const YAML::Node &config_node);
        bool InitFilter(string filter_user, shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node);

    private:
    
        string data_path_ = "";

        /*继承的滤波方法*/
        shared_ptr<CloudFilterInterface> frame_filter_ptr_;
        shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
        shared_ptr<CloudFilterInterface> display_filter_ptr_;
         /*继承的配准方法*/
        shared_ptr<RegistrationInterface> registration_ptr_;

        /*点云地图指针*/
        CloudData::CLOUD_PTR local_map_ptr_;
        CloudData::CLOUD_PTR global_map_ptr_;
        CloudData::CLOUD_PTR result_map_ptr_;

        /*初始位姿*/
        Eigen::Matrix4f  init_pose_=Eigen::Matrix4f::Identity();
        /*当前帧*/
        Frame current_frame_;
        /*地图容器*/
        deque<Frame> local_map_frames_;
        deque<Frame> global_map_frames_;

        int local_frame_num_=20;
        float key_frame_distance_=2.0;

        bool has_new_local_map_=false;
        bool has_new_global_map_=false;

    };
}
#endif
