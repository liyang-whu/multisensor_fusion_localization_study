/*
 * @Description: 一些debug自定义小工具
 * @Author: Robotics gang
 * @note  modified from Ren Qian
 * @Date: 2022-10-02
 */
#include <ros/ros.h>
#include "glog/logging.h"

namespace debug_tools
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
                static void Debug_Info(const std::string str,const std::string value1);
                static void Debug_Info(const std::string str, const double value1);
                static void Debug_Info(const std::string str, const double value1, const double value2);
                static void Debug_Info(const std::string str, const double value1, const double value2, const double value3);
                static void Debug_Info(const std::string str, const double value1, const double value2, const double value3,const double value4);
                static void Debug_Error(const std::string str);
        };

}