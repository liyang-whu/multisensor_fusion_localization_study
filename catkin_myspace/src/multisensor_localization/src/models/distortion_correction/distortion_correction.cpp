#include "../../../include/models/distortion_correction/distortion_correction.hpp"

namespace multisensor_localization
{
    /**
     * @brief 设置运动参数
     * @note
     * @todo
     **/
    void DistortionCorrection::SetMotionParam(float scan_period, VelocityData velocity_data)
    {
        scan_period_ = scan_period;
        linear_velocity_ << velocity_data.linear_velocity_.x, velocity_data.linear_velocity_.y, velocity_data.linear_velocity_.z;
        angular_velocity_ << velocity_data.angular_velocity_.x, velocity_data.angular_velocity_.y, velocity_data.angular_velocity_.z;
    }

    /**
     * @brief 纠正点云信息
     * @note
     * @todo
     **/
    bool DistortionCorrection::AdjustCloud(CloudData::CLOUD_PTR &input_cloud_ptr, CloudData::CLOUD_PTR &output_cloud_ptr)
    {
        CloudData::CLOUD_PTR origin_cloud_ptr(new CloudData::CLOUD(*input_cloud_ptr));
        output_cloud_ptr->points.clear();

        float orientation_space = 2.0 * M_PI;
        float delete_space = 0;//5.0 * M_PI / 180.0;
        float start_orientation = atan2(origin_cloud_ptr->points[0].y,
                                        origin_cloud_ptr->points[0].x);
        Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f rotate_matrix = t_V.matrix();

        Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
        transform_matrix.block<3, 3>(0, 0) = rotate_matrix.inverse();
        pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);

        linear_velocity_ = rotate_matrix.inverse() * linear_velocity_;
        angular_velocity_ = rotate_matrix.inverse() * angular_velocity_;

        for (size_t point_index = 1; point_index < origin_cloud_ptr->points.size(); point_index++)
        {
            float orientation = atan2(origin_cloud_ptr->points[point_index].y,
                                      origin_cloud_ptr->points[point_index].x);
            orientation += orientation < 0.0 ? 2.0 * M_PI : 0.0;

            if (orientation < delete_space || 2 * M_PI - orientation < delete_space)
                continue;

            float real_time = fabs(orientation) / orientation_space * scan_period_ - scan_period_ / 2.0;

            Eigen::Vector3f origin_point(origin_cloud_ptr->points[point_index].x,
                                         origin_cloud_ptr->points[point_index].y,
                                         origin_cloud_ptr->points[point_index].z);

            Eigen::Matrix3f current_matrix = UpdateMatrix(real_time);
            Eigen::Vector3f rotated_point = current_matrix * origin_point;
            Eigen::Vector3f adjust_point = rotated_point + linear_velocity_ * real_time;

            CloudData::POINT point;
            point.x = adjust_point(0);
            point.y = adjust_point(1);
            point.z = adjust_point(2);
            output_cloud_ptr->points.push_back(point);
        }
        pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());
        return true;
    }

    Eigen::Matrix3f DistortionCorrection::UpdateMatrix(float real_time)
    {
        Eigen::Vector3f angle = angular_velocity_ * real_time;
        Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());

        Eigen::AngleAxisf t_V;
        t_V = t_Vz * t_Vy * t_Vx;
        return t_V.matrix();
    }

} // multisensor_localization