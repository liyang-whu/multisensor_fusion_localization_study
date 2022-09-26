/*
 * @Description: front_end_flow 前端里程计算法
 * @Author: robotics 港
 * @Date: 2022-9-26
 * @Note: Modifiled from Ren Qian (github.com/Little-Potato-1990/localization_in_auto_driving)
 */

#ifndef _FRONT_END_H
#define _FRONT_END_H

//通用头文件
#include "../../head.hpp"
//传感器数据类型
#include "../include/sensor_data/cloud_data.hpp"
//前端配准
#include "../include/models/registration/registration_interface.hpp"
#include "../include/models/registration/ndt_registration.hpp"
//点云滤波
#include "../include/models/cloud_filter/cloud_filter_interface.hpp"
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
        FrontEnd();
  
        bool Update(const CloudData &cloud_data, Eigen::Matrix4f &cloud_pose);
        bool SetInitPose(const Eigen::Matrix4f &init_pose);
        bool GetCurrentScan(CloudData::CLOUD_PTR &current_map_ptr);
        bool GetNewLocalMap(CloudData::CLOUD_PTR &local_map_ptr);
        bool GetNewGlobalMap(CloudData::CLOUD_PTR &global_map_ptr);
        bool SaveMap();

    private:
        bool InitWithConfig();
        bool InitDataPath(const YAML::Node &config_node);
        bool InitFilter(string filter_user, shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node);
        bool InitRegistration(shared_ptr<RegistrationInterface> &registeration_ptr, const YAML::Node &config_node);
        bool InitLocalMap(const YAML::Node &config_node);

        bool UpdateNewFrame(const Frame &new_key_frame);
    
     
      

    private:
        /*数据存放路径*/
        string data_path_ = "";
        /*滤波器方法指针*/
        shared_ptr<CloudFilterInterface> frame_filter_ptr_;
        shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
        shared_ptr<CloudFilterInterface> display_filter_ptr_;
        /*点云配准方法指针*/
        shared_ptr<RegistrationInterface> registration_ptr_;

        /*点云地图指针*/
        CloudData::CLOUD_PTR local_map_ptr_;
        CloudData::CLOUD_PTR global_map_ptr_;
        CloudData::CLOUD_PTR result_map_ptr_;
        /*初始位姿*/
        Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

        /*当前帧*/
        Frame current_frame_;
        /*地图队列*/
        deque<Frame> local_map_frames_;
        deque<Frame> global_map_frames_;
        /*局部地图参数*/
        int local_frame_num_ = 20;
        float key_frame_distance_ = 2.0;
        /*新地图标志位*/
        bool has_new_local_map_ = false;
        bool has_new_global_map_ = false;
    };
} // namespace multisensor_localization
#endif
