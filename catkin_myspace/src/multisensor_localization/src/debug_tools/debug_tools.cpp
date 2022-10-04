/*
 * @Description:debug小工具
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:
 * @Date: 2022-10-03
 */

#include "../../include/debug_tools/debug_tools.hpp"

namespace multisensor_localization
{
    /**
     * @brief 终端输出提示
     * @note 
     * @todo
     **/
    void DebugTools::Debug_Info(std::string str)
    {
        std::cout << std::endl
                  << fontColorGreen << str << fontColorReset << std::endl;
    }

    /**
     * @brief 终端输出警告
     * @note
     * @todo
     **/
    void DebugTools::Debug_Warn(const std::string str)
    {
        std::cout<< std::endl
                  << fontColorYellow << str << fontColorReset << std::endl;
    }

    /**
     * @brief 终端输出错误
     * @note
     * @todo
     **/
    void DebugTools::Debug_Error(std::string str)
    {
        std::cout<< std::endl
                  << fontColorRed << str << fontColorReset << std::endl;
        ROS_BREAK();
    }

} // namespace multisensor_localization
