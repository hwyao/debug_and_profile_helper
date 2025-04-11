#ifndef DEBUG_AND_PROFILE_HELPER__HELPER_MACROS_
#define DEBUG_AND_PROFILE_HELPER__HELPER_MACROS_

#include "debug_and_profile_helper/config.h"

#include <string>
#include <chrono>
#include "debug_and_profile_helper/helper_class.hpp"

#include <iostream>

//#define DBGNPROF_USE_FILE
//#define DBGNPROF_USE_ROS
//#define DBGNPROF_ENABLE_DEBUG
//#define DBGNPROF_ENABLE_PROFILE

// Prepare logger instances and time points in the anonymous namespace.
#if defined(DBGNPROF_ENABLE_DEBUG) || defined(DBGNPROF_ENABLE_PROFILE)
    namespace{
        #ifdef DBGNPROF_USE_FILE
            debug_and_profile_helper::LoggerFile& loggerFile_ref__ = debug_and_profile_helper::LoggerFile::getInstance();
        #endif // DBGNPROF_DISABLE_FILE
        #ifdef DBGNPROF_USE_ROS
            debug_and_profile_helper::LoggerROS& loggerROS_ref__ = debug_and_profile_helper::LoggerROS::getInstance();
        #endif // DBGNPROF_USE_ROS
        std::chrono::steady_clock::time_point start_time__;    
        std::chrono::steady_clock::time_point end_time__;

        bool is_clock_started__ = false;
        int clock_start_at_which_line__ = -1;
    }
#endif // DBGNPROF_ENABLE_DEBUG || DBGNPROF_ENABLE_PROFILE

// Define macros aliases for either File or ROS logging, depending on the DBGNPROF_USE_ROS variable.
#if defined(DBGNPROF_USE_ROS)        
    #define DBGNPROF_LOG          DBGNPROF_LOG_TO_ROS
    #define DBGNPROF_STOP_CLOCK   DBGNPROF_STOP_CLOCK_TO_ROS
#else 
    #define DBGNPROF_LOG          DBGNPROF_LOG_TO_FILE
    #define DBGNPROF_STOP_CLOCK   DBGNPROF_STOP_CLOCK_TO_FILE
#endif // DBGNPROF_USE_ROS && DBGNPROF_DEFAULT_ROS

// if DBGNPROF_ENABLE_DEBUG is defined, define the logging macros.
#ifdef DBGNPROF_ENABLE_DEBUG
    #ifdef DBGNPROF_USE_FILE
        #define DBGNPROF_LOG_TO_FILE(name, data) \
            do {\
                if(is_clock_started__==true)\
                {throw std::runtime_error("DBGNPROF_helper: File " + std::string(__FILE__) + " line " + std::to_string(__LINE__) +\
                                ": Clock is running (line " + std::to_string(clock_start_at_which_line__) + "), should not log data.");}\
                std::string debug_name = "DBG_" + std::string(name);\
                loggerFile_ref__.log(debug_name, data);\
            } while (false);
    #endif // DBGNPROF_USE_ROS

    #ifdef DBGNPROF_USE_ROS
        #define DBGNPROF_LOG_TO_ROS(name, data) \
            do {\
                if(is_clock_started__==true)\
                {ROS_ERROR_STREAM_ONCE("DBGNPROF_helper: File " << __FILE__ << " by " << name << " line " << __LINE__ <<\
                                ": Clock is running (line " << clock_start_at_which_line__ << "), should not log data.");}\
                static std::shared_ptr<ros::Publisher> pub_ptr = nullptr;\
                std::string debug_name = "DBG_" + std::string(name);\
                loggerROS_ref__.log(pub_ptr, debug_name, data);\
            } while (false);
    #endif // DBGNPROF_USE_ROS
#endif // DBGNPROF_ENABLE_DEBUG

// if DBGNPROF_ENABLE_PROFILE is defined, define the clock macros.
#ifdef DBGNPROF_ENABLE_PROFILE
    #if defined(DBGNPROF_USE_FILE)
        #define DBGNPROF_START_CLOCK \
            do {\
                if(is_clock_started__==true)\
                {throw std::runtime_error("DBGNPROF_helper: File " + std::string(__FILE__) + " line " + std::to_string(__LINE__) +\
                                ": Clock is running (line " + std::to_string(clock_start_at_which_line__) + "), should not iteratively start the clock.");}\
                start_time__ = std::chrono::steady_clock::now();\
                is_clock_started__ = true;\
                clock_start_at_which_line__ = __LINE__;\
            } while (false);
    #elif defined(DBGNPROF_USE_ROS)
        #define DBGNPROF_START_CLOCK \
            do {\
                if(is_clock_started__==true)\
                {ROS_ERROR_STREAM_ONCE("DBGNPROF_helper: File " << __FILE__ << " line " << __LINE__ <<\
                        ": Clock is running (line " << clock_start_at_which_line__ << "), should not iteratively start the clock.");}\
                start_time__ = std::chrono::steady_clock::now();\
                is_clock_started__ = true;\
                clock_start_at_which_line__ = __LINE__;\
            } while (false);
    #endif // DBGNPROF_USE_FILE

    #ifdef DBGNPROF_USE_FILE
        #define DBGNPROF_STOP_CLOCK_TO_FILE(name) \
            do {\
                if(is_clock_started__==false)\
                {throw std::runtime_error("DBGNPROF_helper: File " + std::string(__FILE__) + " line " + std::to_string(__LINE__) + ": No Clock started.");}\
                end_time__ = std::chrono::steady_clock::now();\
                std::string clock_name = "CLK_" + std::string(name);\
                loggerFile_ref__.log(clock_name, std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>\
                                                (end_time__ - start_time__).count()));\
                is_clock_started__ = false;\
                clock_start_at_which_line__ = -1;\
            } while (false);
    #endif // DBGNPROF_USE_FILE

    #ifdef DBGNPROF_USE_ROS
        #define DBGNPROF_STOP_CLOCK_TO_ROS(name) \
            do {\
                if(is_clock_started__==false)\
                {ROS_ERROR_STREAM_ONCE("DBGNPROF_helper: File " << __FILE__ << " by " << name << " line " << __LINE__ << ": No Clock started.");}\
                end_time__ = std::chrono::steady_clock::now();\
                static std::shared_ptr<ros::Publisher> pub_ptr = nullptr;\
                std::string clock_name = "CLK_" + std::string(name);\
                loggerROS_ref__.log(pub_ptr, clock_name, std::chrono::duration_cast<std::chrono::nanoseconds>\
                                                (end_time__ - start_time__).count());\
                is_clock_started__ = false;\
                clock_start_at_which_line__ = -1;\
            } while (false);
    #endif // DBGNPROF_USE_ROS
#endif // DBGNPROF_ENABLE_PROFILE

// If e macros are not defined from the upper section, define them as empty.
#ifndef DBGNPROF_LOG_TO_FILE
    #define DBGNPROF_LOG_TO_FILE
#endif // DBGNPROF_LOG_TO_FILE

#ifndef DBGNPROF_LOG_TO_ROS
    #define DBGNPROF_LOG_TO_ROS
#endif // DBGNPROF_LOG_TO_ROS

#ifndef DBGNPROF_START_CLOCK
    #define DBGNPROF_START_CLOCK
#endif // DBGNPROF_START_CLOCK

#ifndef DBGNPROF_STOP_CLOCK_TO_FILE
    #define DBGNPROF_STOP_CLOCK_TO_FILE
#endif // DBGNPROF_STOP_CLOCK_TO_FILE

#ifndef DBGNPROF_STOP_CLOCK_TO_ROS
    #define DBGNPROF_STOP_CLOCK_TO_ROS
#endif // DBGNPROF_STOP_CLOCK_TO_ROS

#endif // DEBUG_AND_PROFILE_HELPER__HELPER_MACROS_