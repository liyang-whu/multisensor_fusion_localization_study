/*
 * @Description: 前端里程计节点
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
//数据处理流程控制


using namespace multisensor_localization;

int main(int argc, char **argv)
{
    /*ros系统配置*/
    ros::init(argc, argv, "data_pretreat_node");
    ros::NodeHandle nh;

    /*glog配置*/
    google::InitGoogleLogging(argv[0]);
    std::string path = ros::package::getPath("multisensor_localization");
    FLAGS_log_dir = path + "/Log";
    FLAGS_alsologtostderr = 1;

    /*数据预处理流程指针*/
    //std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatFlow>(nh);

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
       // data_pretreat_flow_ptr->Run();
        rate.sleep();
    }
    return 0;
}
