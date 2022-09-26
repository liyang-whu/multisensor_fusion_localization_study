//通用库文件
#include "../include/front_end/front_end_flow.hpp"
#include "multisensor_localization/saveMap.h"

using namespace multisensor_localization;
shared_ptr<FrontEndFlow> front_end_flow_ptr;

bool SaveMapCallBack(saveMap::Request &request, saveMap::Response &response)
{

    response.succeed = front_end_flow_ptr->SaveMap();
    LOG(INFO) << endl
              << fontColorGreen << ">> >> >> >> >> debug point >> >> >> >> >>" << endl
              << fontColorYellow << "全局地图保存完成" << fontColorReset << endl
              << fontColorGreen << "<< << << << <<  debug point << << << << <<" << endl
              << endl;
    return response.succeed;
}

int main(int argc, char **argv)
{
    /* glog配置 */
    google::InitGoogleLogging(argv[0]);
    string path = ros::package::getPath("multisensor_localization");
    FLAGS_log_dir = path + "/log";
    FLAGS_alsologtostderr = 1;
    boost::filesystem::remove_all(FLAGS_log_dir);
    boost::filesystem::create_directory(FLAGS_log_dir);
    /*节点配置*/
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    const int test_time = 100;
    ros::Rate rate(test_time);
    /*前端里程计初始化*/
    front_end_flow_ptr = make_shared<FrontEndFlow>(nh);
    /*生成全局地图*/
    ros::ServiceServer service = nh.advertiseService("save_map", SaveMapCallBack);

    while (ros::ok())
    {
        ros::spinOnce();
        front_end_flow_ptr->Run();
        // rate.sleep();
    }
    return 0;
}
