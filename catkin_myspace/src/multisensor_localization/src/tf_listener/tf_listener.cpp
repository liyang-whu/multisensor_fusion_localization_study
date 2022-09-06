#include "../../head.hpp"
#include "../../include/tf_listener/tf_listener.hpp"

namespace multisensor_localization
{
    // tf监听初始化
    TfListener::TfListener(ros::NodeHandle &nh, string target_frame_id, string source_frame_id)
        : nh_(nh), target_frame_id_(target_frame_id), source_frame_id_(source_frame_id)
    {
    }

    // 查询变换
    bool TfListener::LookUpData(Eigen::Matrix4f &transform_matrix)
    {
        try
        {
            tf::StampedTransform transform;
            /*target to source */
            listener_.lookupTransform(target_frame_id_, source_frame_id_, ros::Time(0), transform);
            TransformToMatrix(transform, transform_matrix);
            return true;
        }
        catch (tf::TransformException &ex)
        {
            return false;
        }
    }

    // tf旋转类型转为旋转矩阵
    bool TfListener::TransformToMatrix(const tf::StampedTransform &transform, Eigen::Matrix4f &transform_matrix)
    {
        /*提取出平移量*/
        Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
        /*提取旋转量*/
        double roll, pitch, yaw;
        tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
        Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());

        transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
        return true;
    }
}