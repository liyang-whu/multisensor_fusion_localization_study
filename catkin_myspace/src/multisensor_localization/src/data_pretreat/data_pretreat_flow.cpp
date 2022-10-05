/*
 * @Description:传感器数据预处理
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#include "../../include/data_pretreat/data_pretreat_flow.hpp"
namespace multisensor_localization
{

    /**
     * @brief 传感器数据流初始化
     * @note
     * @todo
     **/
    DataPretreatFlow::DataPretreatFlow(ros::NodeHandle &nh)
    {
        /*导入参数文件*/
        std::string config_file_path = ros::package::getPath("multisensor_localization") + "/config/data_pretreat.yaml";
        config_node_ = YAML::LoadFile(config_file_path);
        /*话题订阅*/
        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, config_node_["cloud_sub_topic"].as<std::string>(), 1e5);
        imu_sub_ptr_ = std::make_shared<ImuSubscriber>(nh, config_node_["imu_sub_topic"].as<std::string>(), 1e5);
        velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, config_node_["velocity_sub_topic"].as<std::string>(), 1e5);
        gnss_sub_ptr_ = std::make_shared<GnssSubscriber>(nh, config_node_["gnss_sub_topic"].as<std::string>(), 1e5);
        /*话题发送*/
        cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/synced_cloud", "/velo_link", 100);
        gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/map", "/velo_link", 100);
        origin_pub_ptr_ = std::make_shared<OriginPublisher>(nh, "ref_point_wgs84", 100, "map");
        /*畸变矫正*/

        /*just a test*/
    }

    /**
     * @brief 数据预处理逻辑运行
     * @note
     * @todo
     **/
    bool DataPretreatFlow::Run()
    {
        /*读取数据并同步时间*/
        if (!ReadData())
        {
            return false;
        }
        /*多传感器空间标定*/
        if (!InitCalibration())
        {
            return false;
        }
        /*初始化gnss东北天坐标系*/
        if (!InitGNSS())
        {
            return false;
        }
        while (HasData())
        {
            if (!ValidData())
                continue;
            TransformData();
            PublishData();
        }
        return true;
    }

    /**
     * @brief 多传感器时间同步
     * @note 非硬件同步触发时软件插值同步
     * @todo
     **/
    bool DataPretreatFlow::ReadData()
    {
        /*待同步队列*/
        static std::deque<ImuData> unsynced_imu_data_buff;
        static std::deque<VelocityData> unsynced_velocity_data_buff;
        static std::deque<GnssData> unsynced_gnss_data_buff;
        /*传感器数据读取*/
        cloud_sub_ptr_->ParseData(cloud_data_buff_);
        imu_sub_ptr_->ParseData(unsynced_imu_data_buff);
        velocity_sub_ptr_->ParseData(unsynced_velocity_data_buff);
        gnss_sub_ptr_->ParseData(unsynced_gnss_data_buff);

        if (cloud_data_buff_.size() == 0)
            return false;

        /*以雷达数据为基准做数据同步*/
        double cloud_time = cloud_data_buff_.front().time_stamp_;
        bool is_valid_imu = ImuData::SyncData(unsynced_imu_data_buff, imu_data_buff_, cloud_time);
        bool is_valid_velocity = VelocityData::SyncData(unsynced_velocity_data_buff, velocity_data_buff_, cloud_time);
        bool is_valid_gnss = GnssData::SyncData(unsynced_gnss_data_buff, gnss_data_buff_, cloud_time);

        /*起始阶段时间戳是否对齐*/
        static bool sensor_inited = false;
        if (!sensor_inited)
        {
            if (!is_valid_imu || !is_valid_velocity || !is_valid_gnss)
            {
                cloud_data_buff_.pop_front();
                return false;
            }
            DebugTools::Debug_Info("多传感器时间同步已完成");
            sensor_inited = true;
        }

        return true;
    }

    /**
     * @brief 多传感器空间标定
     * @note gnss与LIO必须在线标定
     * @todo YAML 读参
     **/
    bool DataPretreatFlow::InitCalibration()
    {
        static bool calibration_finished = false;
        if (!calibration_finished)
        {
            std::string config_file_path = ros::package::getPath("multisensor_localization") + "/config/data_pretreat.yaml";
            YAML::Node config_node = YAML::LoadFile(config_file_path);
            lidar_to_imu_ = Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(config_node_["calibration_param"]["lidar_to_imu"].as<std::vector<float>>().data());
            LOG(INFO) << std::endl
                      << "[lidar_to_imu]" << std::endl
                      << lidar_to_imu_ << std::endl;

            DebugTools::Debug_Info("多传感器空间标定已完成");
            calibration_finished = true;
        }
        return calibration_finished;
    }

    /**
     * @brief 初始化东北天坐标系
     * @note
     * @todo
     **/
    bool DataPretreatFlow::InitGNSS()
    {
        static bool gnss_inited = false;
        if (!gnss_inited)
        {
            GnssData gnss_data = gnss_data_buff_.front();
            gnss_data.InitOriginPosition();
            gnss_inited = true;
            DebugTools::Debug_Info("东北天坐标系初始化已完成");
            LOG(INFO) << std::endl
                      << "[ENU origin point]" << std::endl
                      << "longitude \t" << gnss_data.longitude_ << std::endl
                      << "latitude \t" << gnss_data.latitude_ << std::endl;

            origin_pub_ptr_->Publish(gnss_data);
        }
        return gnss_inited;
    }

    /**
     * @brief 检查各队列数据是否存在
     * @note
     * @todo
     **/
    bool DataPretreatFlow::HasData()
    {
        if (cloud_data_buff_.size() == 0)
            return false;
        if (imu_data_buff_.size() == 0)
            return false;
        if (velocity_data_buff_.size() == 0)
            return false;
        if (gnss_data_buff_.size() == 0)
            return false;

        return true;
    }

    /**
     * @brief 检查数据有效性
     * @note 主要检查时间戳同步程度
     * @todo
     **/
    bool DataPretreatFlow::ValidData()
    {
        /*取出头数据*/
        current_cloud_data_ = cloud_data_buff_.front();
        current_imu_data_ = imu_data_buff_.front();
        current_velocity_data_ = velocity_data_buff_.front();
        current_gnss_data_ = gnss_data_buff_.front();
        /*时间戳同步程度校验 时间戳滞后数据丢掉*/
        double diff_imu_time = current_cloud_data_.time_stamp_ - current_imu_data_.time_stamp_;
        double diff_velocity_time = current_cloud_data_.time_stamp_ - current_velocity_data_.time_stamp_;
        double diff_gnss_time = current_cloud_data_.time_stamp_ - current_gnss_data_.time_stamp_;

        if (diff_imu_time < -0.05 || diff_velocity_time < -0.05 || diff_gnss_time < -0.05)
        {
            cloud_data_buff_.pop_front();
            return false;
        }
        if (diff_imu_time > 0.05)
        {
            imu_data_buff_.pop_front();
            return false;
        }

        if (diff_velocity_time > 0.05)
        {
            velocity_data_buff_.pop_front();
            return false;
        }

        if (diff_gnss_time > 0.05)
        {
            gnss_data_buff_.pop_front();
            return false;
        }
        /*弹出已取的头数据*/
        cloud_data_buff_.pop_front();
        imu_data_buff_.pop_front();
        velocity_data_buff_.pop_front();
        gnss_data_buff_.pop_front();

        LOG(INFO) << std::endl
                  << "[time_stamp]" << std::endl
                  << "current_cloud_data \t " << std::fixed << current_cloud_data_.time_stamp_ << std::endl
                  << "current_imu_data \t" << std::fixed << current_imu_data_.time_stamp_ << std::endl
                  << "current_velocity_data \t" << std::fixed << current_velocity_data_.time_stamp_ << std::endl
                  << "current_gnss_data \t" << std::fixed << current_gnss_data_.time_stamp_ << std::endl;

        return true;
    }

    /**
     * @brief 数据坐标系转换
     * @note
     * @todo
     **/
    bool DataPretreatFlow::TransformData()
    {
        /*gnss位姿转换*/
        gnss_pose_ = Eigen::Matrix4f::Identity();
        current_gnss_data_.UpdateXYZ();
        gnss_pose_(0, 3) = current_gnss_data_.local_E_;
        gnss_pose_(1, 3) = current_gnss_data_.local_N_;
        gnss_pose_(2, 3) = current_gnss_data_.local_U_;
        gnss_pose_.block<3, 3>(0, 0) = current_imu_data_.OrientationToMatrix();
        gnss_pose_ *= lidar_to_imu_;
        /*点云畸变矫正*/
        // current_velocity_data_.TransformCoordinate(lidar_to_imu_.inverse());
        //! TODO 畸变矫正变换似乎有问题 暂时跳过
        return true;
    }

    /**
     * @brief 数据发布到前端
     * @note
     * @todo
     **/
    bool DataPretreatFlow::PublishData()
    {
        cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr_, current_cloud_data_.time_stamp_);
        gnss_pub_ptr_->Publish(gnss_pose_, current_gnss_data_.time_stamp_);

        return true;
    }
} // namespace multisensor_localization
