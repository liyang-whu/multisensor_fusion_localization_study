/*
 * @Description: back end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:31:22
 */

#include "../../../include/mapping/back_end/back_end.hpp"
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/package.h>

namespace multisensor_localization
{
    /**
     * @brief 后面端流程控制初始化
     * @note
     * @todo
     **/
    BackEnd::BackEnd()
    {
        /*参数文件读取*/
        std::string config_file_path = ros::package::getPath("multisensor_localization") + "/config/back_end.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);
        /*参数配置*/
        ConfigFrame(config_node);
        ConfigGraphOptimizer(config_node);
        ConfigDataPath(config_node);
    }

    /**
     * @brief 关键帧参数配置
     * @note
     * @todo
     **/
    bool BackEnd::ConfigFrame(const YAML::Node &config_node)
    {
        key_frame_distance_=config_node["key_frame_distance"].as<float>();
        return true;
    }

    /**
     * @brief   图优化参数配置
     * @note
     * @todo
     **/
    bool BackEnd::ConfigGraphOptimizer(const YAML::Node &config_node)
    {
        
    }

    /**
     * @brief   配置数据保存路径
     * @note
     * @todo
     **/
    bool BackEnd::ConfigDataPath(const YAML::Node &config_node)
    {
    }

}
