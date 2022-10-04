/*
 * @Description: velocity 数据
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:27:40
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_HPP_

#include <deque>
#include <Eigen/Dense>
#include <glog/logging.h>

namespace multisensor_localization
{
  class VelocityData
  {
  public:
    struct LinearVelocity
    {
      double x = 0.0, y = 0.0, z = 0.0;
    };

    struct AngularVelocity
    {
      double x = 0.0, y = 0.0, z = 0.0;
    };

    double time_stamp_ = 0.0;
    LinearVelocity linear_velocity_;
    AngularVelocity angular_velocity_;

  public:
    static bool SyncData(std::deque<VelocityData> &unsynced_data_buff,
                         std::deque<VelocityData> &synced_data_buff,
                         double sync_time);
    void TransformCoordinate(Eigen::Matrix4f transform_matrix);
  };
}
#endif