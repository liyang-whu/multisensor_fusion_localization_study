#include "../../include/tools/file_manager.h"

namespace multisensor_localization
{

    /**
    @brief 创建文件
    @note
    @todo
    **/
    bool FileManager::CreateFile(ofstream &ofs, string file_path)
    {
        ofs.open(file_path.c_str(), ios::app);
        if (!ofs)
        {
            LOG(INFO) << endl
                      << fontColorRedBold << "无法生成文件" << endl
                      << fontColorReset << endl;
            ROS_BREAK();
            return false;
        }
        return true;
    }

    /**
    @brief 创建文件夹
    @note
    @todo
    **/
    bool FileManager::CreateDirectory(string directory_path)
    {
        if (!boost::filesystem::is_directory(directory_path))
        {
            boost::filesystem::create_directory(directory_path);
        }
        if (!boost::filesystem::is_directory(directory_path))
        {
            LOG(INFO) << endl
                      << fontColorRedBold << "无法生成文件" << endl
                      << fontColorReset << endl;
            ROS_BREAK();
        }
        return true;
    }

}