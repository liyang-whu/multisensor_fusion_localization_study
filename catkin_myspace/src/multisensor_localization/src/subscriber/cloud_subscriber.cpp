#include "../../head.hpp"
#include "../../include/subscriber/cloud_subscriber.hpp"

namespace multisensor_localization
{

    // 初始化点云接收
    CloudSubscriber::CloudSubscriber(ros::NodeHandle &nh, string topic_name, size_t buff_size) : nh_(nh)
    {
        subscriber_ = nh.subscribe(topic_name, buff_size, &CloudSubscriber::MsgCallback, this);
    }

    //点云回调函数 
    void CloudSubscriber::MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr)
    {
        CloudData cloud_data;
        /*时间戳因为要转换成秒为单位所以单独拷贝*/
        cloud_data.time_stamp_ = cloud_msg_ptr->header.stamp.toSec(); 
        /*将点云消息从ROS类型转换为PCL类型*/
        pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr_));
        new_cloud_data_.push_back(cloud_data);
    }

    //数据解析 类似于串口缓冲区机制
    void CloudSubscriber::ParseData(deque<CloudData> &cloud_data_buff)
    {
        if (new_cloud_data_.size() > 0)
        {
            cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end()); /*末尾追加数据 经典写法*/
            new_cloud_data_.clear();
        }
    }
} // namespace multisensor_localization