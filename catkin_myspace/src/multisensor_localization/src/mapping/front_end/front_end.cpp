/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:53:06
 */

//文件读写
#include <fstream>
#include <boost/filesystem.hpp>
// ros库
#include <ros/package.h>
// pcl库
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
//前端算法
#include "../../../include/mapping/front_end/front_end.hpp"
//匹配方法
#include "../../../include/models/registration/ndt_registration.hpp"
//体素滤方法
#include "../../../include/models/cloud_filter/voxel_filter.hpp"
// debug方式
#include "../../../include/debug_tools/debug_tools.hpp"

#include <glog/logging.h>

namespace multisensor_localization
{

    /**
     * @brief 前端流程控制初始化
     * @note 订阅点云信息 发布激光里程计
     * @todo
     **/
    FrontEnd::FrontEnd()
        : local_map_ptr_(new CloudData::CLOUD())
    {
        /*读取YAML参数*/
        std::string config_file_path = ros::package::getPath("multisensor_localization") + "/config/front_end.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);
        /*参数配置*/
        ConfigFrame(config_node);
        ConfigRegistrationMethod(registration_ptr_, config_node);
        ConfigFilterMethod("local_map", local_map_filter_ptr_, config_node);
        ConfigFilterMethod("frame", frame_filter_ptr_, config_node);
    }

    /**
     * @brief  关键帧配置
     * @note
     * @todo
     **/
    bool FrontEnd::ConfigFrame(const YAML::Node &config_node)
    {
        key_frame_distance_ = config_node["key_frame_distance"].as<float>();
        local_frame_num_ = config_node["local_frame_num"].as<int>();
        return true;
    }

    /**
     * @brief  配置点云配准方法
     * @note
     * @todo
     **/
    bool FrontEnd::ConfigRegistrationMethod(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &config_node)
    {
        std::string registration_method = config_node["registration_method"].as<std::string>();

        if (registration_method == "NDT")
        {
            registration_ptr = std::make_shared<NdtRegistration>(config_node[registration_method]);
            LOG(INFO) << std::endl
                      << "[registration_method]" << std::endl
                      << registration_method << std::endl;
        }
        else
        {
            DebugTools::Debug_Error("无对应匹配方法");
            return false;
        }

        return true;
    }

    /**
     * @brief  配置点云滤波方法
     * @note
     * @todo
     **/
    bool FrontEnd::ConfigFilterMethod(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node)
    {
        std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();

        if (filter_mothod == "voxel_filter")
        {
            filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
            LOG(INFO) << std::endl
                      << "[filter_mothod]" << std::endl
                      << filter_mothod << std::endl;
        }
        else
        {
            DebugTools::Debug_Error("无对应滤波方法");
            return false;
        }

        return true;
    }

    /**
     * @brief  设置初始位姿
     * @note
     * @todo
     **/
    bool FrontEnd::SetInitPose(const Eigen::Matrix4f &init_pose)
    {
        init_pose_ = init_pose;
        return true;
    }

    /**
     * @brief  里程计更新
     * @note
     * @todo
     **/
    bool FrontEnd::UpdateOdometry(const CloudData &cloud_data, Eigen::Matrix4f &cloud_pose)
    {
        /*拷贝时间戳*/
        current_frame_.cloud_data_.time_stamp_ = cloud_data.time_stamp_;
        /*去除无效点*/
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr_, *current_frame_.cloud_data_.cloud_ptr_, indices);
        /*滤波降采样*/
        CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
        frame_filter_ptr_->Filter(current_frame_.cloud_data_.cloud_ptr_, filtered_cloud_ptr);
        /*定义用于运动递推的参数 */
        static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
        static Eigen::Matrix4f last_pose = init_pose_;
        static Eigen::Matrix4f predict_pose = init_pose_;
        static Eigen::Matrix4f last_key_frame_pose = init_pose_;

        /*第一帧处理*/
        if (local_map_frames_.size() == 0)
        {
            current_frame_.pose_ = init_pose_;
            AddNewFrame(current_frame_);
            cloud_pose = current_frame_.pose_; //引用取参值
            return true;
        }

        /*直接匹配(非第一帧)*/
        CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
        registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, current_frame_.pose_);
        cloud_pose = current_frame_.pose_; //引用取参值

        /*更新相邻两帧的相对运动*/
        step_pose = last_pose.inverse() * current_frame_.pose_;
        predict_pose = current_frame_.pose_ * step_pose;
        last_pose = current_frame_.pose_;

        /*是否更新关键帧*/
        if (fabs(last_key_frame_pose(0, 3) - current_frame_.pose_(0, 3)) +
                fabs(last_key_frame_pose(1, 3) - current_frame_.pose_(1, 3)) +
                fabs(last_key_frame_pose(2, 3) - current_frame_.pose_(2, 3)) >
            key_frame_distance_)
        {
            AddNewFrame(current_frame_);
            last_key_frame_pose = current_frame_.pose_;
        }

        return true;
    }

    /**
     * @brief  更新新关键帧
     * @note
     * @todo
     **/
    bool FrontEnd::AddNewFrame(const Frame &new_key_frame)
    {
        /*深拷贝到本地*/
        Frame key_frame = new_key_frame;
        key_frame.cloud_data_.cloud_ptr_.reset(new CloudData::CLOUD(*new_key_frame.cloud_data_.cloud_ptr_));
        CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

        /*更新局部地图*/
        local_map_frames_.push_back(key_frame);
        while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_))
        {
            local_map_frames_.pop_front();
        }
        local_map_ptr_.reset(new CloudData::CLOUD());
        for (size_t i = 0; i < local_map_frames_.size(); ++i)
        {
            pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data_.cloud_ptr_,
                                     *transformed_cloud_ptr,
                                     local_map_frames_.at(i).pose_);

            *local_map_ptr_ += *transformed_cloud_ptr;
        }

        /*更新目标点云*/
        if (local_map_frames_.size() < 10)
        {
            registration_ptr_->SetInputTarget(local_map_ptr_);
        }
        else
        {
            CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
            local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
            registration_ptr_->SetInputTarget(filtered_local_map_ptr);
        }
        return true;
    }

}