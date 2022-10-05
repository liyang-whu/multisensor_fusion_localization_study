/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 */
#ifndef FRONT_END_FRONT_END_HPP_
#define FRONT_END_FRONT_END_HPP_

#include <Eigen/Dense>
#include <deque>
#include "../../sensor_data/cloud_data.hpp"
#include "../../models/cloud_filter/cloud_filter_interface.hpp"
#include "../../models/registration/registration_interface.hpp"

namespace multisensor_localization
{
    class FrontEnd
    {
    public:
        struct Frame
        {
            Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();
            CloudData cloud_data_;
        };

    public:
        FrontEnd();

        bool UpdateOdometry(const CloudData &cloud_data, Eigen::Matrix4f &cloud_pose);
         bool SetInitPose(const Eigen::Matrix4f &init_pose);

    private:
         bool ConfigFrame(const YAML::Node &config_node);
         bool ConfigRegistrationMethod(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &config_node);
         bool ConfigFilterMethod(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node);
         
        bool AddNewFrame(const Frame &new_key_frame);

    private:
        std::string data_path_ = "";

        std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
        std::shared_ptr<RegistrationInterface> registration_ptr_;

        std::deque<Frame> local_map_frames_;

        CloudData::CLOUD_PTR local_map_ptr_;
        Frame current_frame_;

        Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

        float key_frame_distance_ = 2.0;
        int local_frame_num_ = 20;
    };
}
#endif