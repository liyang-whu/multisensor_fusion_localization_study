#include "../../include/sensor_data/gnss_data.hpp"

namespace multisensor_localization
{
    GeographicLib::LocalCartesian GnssData::geo_converter_; /*静态变量类内定义类外初始化*/
    bool GnssData::origin_position_inited_ = false;

    void GnssData::InitOriginPosition()
    {
        geo_converter_.Reset(latitude_, longtitude_, altitude_);
        origin_position_inited_ = true;
    }
    void GnssData::UpdateXYZ()
    {
        if (origin_position_inited_ == false)
        {
            //cout<<"东北天坐标系未初始化"<<endl;
        }
        geo_converter_.Forward(latitude_, longtitude_, altitude_, local_E_, local_N_, local_U_);
    }
}
