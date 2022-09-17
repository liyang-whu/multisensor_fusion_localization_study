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



#endif

