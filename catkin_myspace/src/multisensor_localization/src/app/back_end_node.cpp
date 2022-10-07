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
//后端流程控制流程
#include "../../include/mapping/back_end/back_end.hpp"
#include "../../include/mapping/back_end/back_end_flow.hpp"

using namespace multisensor_localization;

int main(int argc, char **argv)
{
    /*ros系统配置*/
    ros::init(argc, argv, "back_end_node");
    ros::NodeHandle nh;

    /*后端优化流程*/
    std::shared_ptr<BackEndFlow> back_end_flow_ptr=std::make_shared<BackEndFlow>();

    /*glog配置*/
    google::InitGoogleLogging(argv[0]);
    std::string path = ros::package::getPath("multisensor_localization");
    FLAGS_log_dir = path + "/Log";
    FLAGS_alsologtostderr = 1;

   
    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
