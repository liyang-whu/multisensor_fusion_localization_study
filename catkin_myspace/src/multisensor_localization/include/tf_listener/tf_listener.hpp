#ifndef _TF_LISTENER_H
#define _TF_LISTENER_H

#include "../../head.hpp"

namespace multisensor_localization
{
    class TfListener
    {
        TfListener(ros::NodeHandle &nh, string target_frame_id, string source_frame_id);
        TfListener() = default;

        bool LookUpData(Eigen::Matrix4f &transform_matrix);

    private:
        bool TransformToMatrix(const tf::StampedTransform &transform, Eigen::Matrix4f &transform_matrix);

    private:
        ros::NodeHandle nh_;
        tf::TransformListener listener_;
        string target_frame_id_;
        string source_frame_id_;
    };
}

#endif