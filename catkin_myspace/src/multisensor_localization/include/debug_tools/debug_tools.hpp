/*
 * @Description: 一些debug自定义小工具
 * @Author: Robotics gang
 * @note  modified from Ren Qian
 * @Date: 2022-10-02
 */
#ifndef DEBUG_TOOLS_HPP_
#define DEBUG_TOOLS_HPP_

#include <ros/ros.h>

namespace multisensor_localization
{

/*终端彩色字体*/
#define fontColorReset "\033[0m"
#define fontColorBlack "\033[30m"
#define fontColorRed "\033[31m"
#define fontColorGreen "\033[32m"
#define fontColorYellow "\033[33m"
#define fontColorBlue "\033[34m"
#define fontColorMagenta "\033[35m"
#define fontColorCyan "\033[36m"
#define fontColorWhite "\033[37m"
#define fontColorBlackBold "\033[1m\033[30m"
#define fontColorRedBold "\033[1m\033[31m"
#define fontColorGreenBold "\033[1m\033[32m"
#define fontColorYellowBold "\033[1m\033[33m"
#define fontColorBlueBold "\033[1m\033[34m"
#define fontColorMagentaBold "\033[1m\033[35m"
#define fontColorCyanBold "\033[1m\033[36m"
#define fontColorWhiteBold "\033[1m\033[37m"

        class DebugTools
        {
        public:
                static void Debug_Info(const std::string str);
                static void Debug_Warn(const std::string str);
                static void Debug_Error(const std::string str);
        };

}

#endif