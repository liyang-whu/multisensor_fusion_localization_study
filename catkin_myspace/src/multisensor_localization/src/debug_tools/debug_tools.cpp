/*
 * @Description:debug小工具
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:
 * @Date: 2022-10-03
 */

#include "../../include/debug_tools/debug_tools.hpp"

namespace debug_tools
{
    /**
     * @brief 终端输出提示
     * @note 以重载形式适配不定参数
     * @todo
     **/
    void DebugTools::Debug_Info(std::string str)
    {
        LOG(INFO) << std::endl
                  << fontColorGreen << str << fontColorReset << std::endl;
    }

    void DebugTools::Debug_Info(const std::string str, const std::string value1)
    {
        LOG(INFO) << std::endl
                  << fontColorGreen << str << fontColorReset << std::endl
                  << fontColorBlue << value1 << fontColorReset << std::endl;
    }

    void DebugTools::Debug_Info(const std::string str, const double value1)
    {
        LOG(INFO) << std::endl
                  << fontColorGreen << str << fontColorReset << std::endl
                  << fontColorBlue << value1 << fontColorReset << std::endl;
    }

    void DebugTools::Debug_Info(const std::string str, const double value1, const double value2)
    {
        LOG(INFO) << std::endl
                  << fontColorGreen << str << fontColorReset << std::endl
                  << fontColorBlue << value1 << fontColorReset << std::endl
                  << fontColorBlue << value2 << fontColorReset << std::endl;
    }

    void DebugTools::Debug_Info(const std::string str, const double value1, const double value2, const double value3)
    {
        LOG(INFO) << std::endl
                  << fontColorGreen << str << fontColorReset << std::endl
                  << fontColorBlue << value1 << fontColorReset << std::endl
                  << fontColorBlue << value2 << fontColorReset << std::endl
                  << fontColorBlue << value3 << fontColorReset << std::endl;
    }

    void DebugTools::Debug_Info(const std::string str, const double value1, const double value2, const double value3, const double value4)
    {
        LOG(INFO) << std::endl
                  << fontColorGreen << str << fontColorReset << std::endl
                  << fontColorBlue << value1 << fontColorReset << std::endl
                  << fontColorBlue << value2 << fontColorReset << std::endl
                  << fontColorBlue << value3 << fontColorReset << std::endl
                  << fontColorBlue << value4 << fontColorReset << std::endl;
    }

    /**
     * @brief 终端输出错误
     * @note 
     * @todo
     **/
    void DebugTools::Debug_Error(std::string str)
    {
        LOG(INFO) << std::endl
                  << fontColorRed << str << fontColorReset << std::endl;
        ROS_BREAK();
    }

}
