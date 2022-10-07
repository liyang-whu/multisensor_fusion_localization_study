/*
 * @Description: back end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:31:22
 */

#ifndef BACK_END_END_FLOW_HPP_
#define BACK_END_END_FLOW_HPP_

#include "../../../include/sensor_data/cloud_data.hpp"

#include <yaml-cpp/yaml.h>

namespace multisensor_localization
{
    class BackEnd
    {
    public:
        BackEnd();

    private:
        bool ConfigFrame(const YAML::Node &config_node);
        bool ConfigGraphOptimizer(const YAML::Node &config_node);
        bool ConfigDataPath(const YAML::Node &config_node);

        float key_frame_distance_ = 2.0;
    };

}

#endif