#include "../../include/sensor_data/pose_data.hpp"

namespace multisensor_localization
{
    /**
     * @brief 提取出旋转矩阵
     * @note
     * @todo
     **/
    Eigen::Quaternionf PoseData::GetQuaternion()
    {
        Eigen::Quaternionf q;
        q=pose_.block<3, 3>(0, 0);
        return q;
    }

} // namespace multisensor_localization
