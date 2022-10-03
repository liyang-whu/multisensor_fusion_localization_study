#include "../../include/subscriber/cloud_subscriber.hpp"

/*
 * @Description: 接收点云消息
 * @Author: Robotics gang
 * @note  modified from Ren Qian
 * @Date: 2022-10-02
 */

namespace multisensor_localization
{

    /**
     * @brief  点云接收初始化
     * @note
     * @todo
     **/
    CloudSubscriber::CloudSubscriber(ros::NodeHandle &nh,
                                     std::string topic_name,
                                     size_t buff_size) : nh_(nh)
    {
        subscriber_ = nh.subscribe(topic_name, buff_size, &CloudSubscriber::MsgCallback, this);
    }

    /**
     * @brief  点云接收回调函数
     * @note
     * @todo
     **/
    void CloudSubscriber::MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr)
    {
        CloudData cloud_data;
        /*时间戳因为要转换成秒为单位所以单独拷贝*/
        cloud_data.time_stamp_ = cloud_msg_ptr->header.stamp.toSec();
        /*将点云消息从ROS类型转换为PCL类型*/
        pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr_));
        new_cloud_data_.push_back(cloud_data);
    }

    /**
     * @brief  读取缓冲区数据
     * @note
     * @todo
     **/
    void CloudSubscriber::ParseData(std::deque<CloudData> &cloud_data_buff)
    {
        if (new_cloud_data_.size() > 0)
        {
            cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end()); /*末尾追加数据 经典写法*/
            new_cloud_data_.clear();
        }
    }
} // namespace multisensor_localization