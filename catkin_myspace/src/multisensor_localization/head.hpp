#ifndef _HEAD_H
#define _HEAD_H

//c++库
#include "bits/stdc++.h"
using namespace std;

//ros melodic 库
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include  "tf/transform_listener.h"
#include "nav_msgs/Odometry.h"
#include  "ros/package.h"

//pcl库(ros melodic自带版本1.8.1 )
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/transforms.h"

//eigen 库(ros melodic自带版本)
#include "Eigen/Dense"

//geographicLib 地理日志库2.1.1
#include "GeographicLib/LocalCartesian.hpp"

//glog库
#include "glog/logging.h"

//yaml-cpp库
#include "yaml-cpp/yaml.h"

//终端输出字体颜色
#define fontColorReset   "\033[0m"
#define fontColorBlack   "\033[30m"      /* Black */
#define fontColorRed     "\033[31m"      /* Red */
#define fontColorGreen   "\033[32m"      /* Green */
#define fontColorYellow  "\033[33m"      /* Yellow */
#define fontColorBlue   "\033[34m"      /* Blue */
#define fontColorMagenta "\033[35m"      /* Magenta */
#define fontColorCyan   "\033[36m"      /* Cyan */
#define fontColorWhite  "\033[37m"      /* White */

#define fontColorBlackBold   "\033[1m\033[30m"      /* Bold Black */
#define fontColorRedBold     "\033[1m\033[31m"      /* Bold Red */
#define fontColorGreenBold   "\033[1m\033[32m"      /* Bold Green */
#define fontColorYellowBold  "\033[1m\033[33m"      /* Bold Yellow */
#define fontColorBlueBold    "\033[1m\033[34m"      /* Bold Blue */
#define fontColorMagentaBold "\033[1m\033[35m"      /* Bold Magenta */
#define fontColorCyanBold    "\033[1m\033[36m"      /* Bold Cyan */
#define fontColorWhiteBold   "\033[1m\033[37m"      /* Bold White */

#endif

