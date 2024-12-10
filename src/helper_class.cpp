/**
 * @file helper_class.cpp
 * @brief Implementation of the LoggerFile and LoggerROS classes.
 * @date 2024-12-08
 * 
 */

#include "debug_and_profile_helper/helper_class.hpp"

#include <chrono>
#include <iomanip>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

namespace debug_and_profile_helper {
    class LoggerFile::pimplData {
    public:
        std::unique_ptr<spdlog::logger> logger;
    };

    void LoggerFile::pimplDataDeleter::operator()(LoggerFile::pimplData* p) {
        delete p;
    }

    LoggerFile::LoggerFile(const std::string& filePath) : data_{ new pimplData() } {
        // generate a log file name based on the current date and time
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");

        // create the log file full path
        std::string fileName = filePath + std::string("log_") + ss.str() + std::string(".txt");

        // initialize the pimplData object with a logger
        data_->logger = std::make_unique<spdlog::logger>("file_logger", std::make_shared<spdlog::sinks::basic_file_sink_mt>(fileName, true));
    }

    void LoggerFile::log() const {
        SPDLOG_LOGGER_INFO(data_->logger, "[empty log message]");
    }

    void LoggerFile::logInternal(const std::string& name, const std::string& data) const {
        SPDLOG_LOGGER_INFO(data_->logger, "{}: {}", name, data);
    }
}

#ifdef USE_ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace debug_and_profile_helper {
    struct LoggerROS::pimplData {
        ros::NodeHandle nh;
        std::string topicPrefix;
        int queue_size;
    };

    void LoggerROS::pimplDataDeleter::operator()(LoggerROS::pimplData* p) {
        delete p;
    }

    LoggerROS::LoggerROS(const std::string& topicPrefix) : data_{ new pimplData() } {

    }

    void LoggerROS::log() const {

    }
} // namespace debug_and_profile_helper
#endif // USE_ROS