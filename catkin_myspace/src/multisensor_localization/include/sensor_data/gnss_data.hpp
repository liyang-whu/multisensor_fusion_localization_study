#ifndef  _GNSS_DATA_H
#define _GNSS_DATA_H

#include "../../head.hpp"

namespace multisensor_localization
{
    class GnssData
    {
    public:
        double time_stamp_ = 0.0;
        double longtitude_ = 0.0; /*经度*/
        double latitude_ = 0.0;   /*纬度*/
        double altitude_ = 0.0;   /*海拔*/
        double local_E_ = 0.0;    /*东北天坐标系东X*/
        double local_N_ = 0.0;    /*东北天坐标系北Y*/
        double local_U_ = 0.0;    /*东北天坐标系上Z*/
        int status_;
        int service_=0;

        private:
        static GeographicLib::LocalCartesian geo_converter_;/*静态变量类内定义类外初始化*/
        static bool  origin_position_inited_;

        public:
        void InitOriginPosition();
        void UpdateXYZ();

    };
}

#endif