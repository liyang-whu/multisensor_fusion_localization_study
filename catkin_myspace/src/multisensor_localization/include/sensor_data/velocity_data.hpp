#ifndef  _VELOCITY_DATA_H
#define _VELOCITY_DATA_H

#include "../../head.hpp"
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
        static bool SyncData(deque<VelocityData> &unsynced_data_buff,
                             deque<VelocityData> &synced_data_buff,
                             double sync_time);
    };

} // namespace multisensor_localization
#endif