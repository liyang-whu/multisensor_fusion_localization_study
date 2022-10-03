/*
 * @Description: 数据预处理的node文件
 * @Author: Robotic Gang
 * @Function:
 * @note  modified from Ren Qian
 * @Date: 2022-10-02
 */

// ros库文件
#include <ros/ros.h>
#include <ros/package.h>
// glog日志库
#include <glog/logging.h>
// debug自定义工具
#include "../../include/debug_tools/debug_tools.hpp"
using namespace debug_tools;

int main(int argc, char **argv)
{
    /*ros系统配置*/
    ros::init(argc, argv, "data_preprocess_node");
    ros::NodeHandle nh;

    /*glog配置*/
    google::InitGoogleLogging(argv[0]);
    std::string path = ros::package::getPath("multisensor_localization");
    FLAGS_log_dir = path + "/Log";
    FLAGS_alsologtostderr = 1;

    /*数据预处理流程指针*/

    ros::Rate rate(100);
    while (ros::ok())
    {
        DebugTools::Debug_Info("循环进行中");

        rate.sleep();
    }
    return 0;
}
