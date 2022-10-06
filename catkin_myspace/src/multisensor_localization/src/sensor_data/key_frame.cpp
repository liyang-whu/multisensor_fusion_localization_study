#include "../../include/sensor_data/key_frame.hpp"

namespace multisensor_localization
{
    /**
     * @brief 提取出旋转矩阵
     * @note
     * @todo
     **/
    Eigen::Quaternionf KeyFrame::GetQuaternion()
    {
        Eigen::Quaternionf q;
        q=pose_.block<3, 3>(0, 0);
        return q;
    }

} // namespace multisensor_localization
