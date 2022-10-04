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
            DebugTools::Debug_Warn("未能完成多传感器时间同步");
            return false;
        }
        /*多传感器空间标定*/
        if (!InitCalibration())
        {
            DebugTools::Debug_Warn("未能完成多传感器空间标定");
            return false;
        }
        /*初始化gnss东北天坐标系*/
        if (!InitGNSS())
        {
            DebugTools::Debug_Warn("gnss东北天坐标系原点已设定");
            return false;
        }
        while (HasData())
        {
            //提取有效数据
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
        std::string config_file_path = ros::package::getPath("multisensor_localization") + "/config/data_pretreat.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);
        static bool calibration_finished = false;
        if (!calibration_finished)
        {
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

} // namespace multisensor_localization
