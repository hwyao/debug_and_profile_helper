/**
 * @file helper_class.cpp
 * @author Haowen Yao
 * @brief Implementation of the LoggerFile and LoggerROS classes.
 * @date 2024-12-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "debug_and_profile_helper/helper_class.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <unordered_set>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

namespace debug_and_profile_helper {
    /**
     * @brief A class to store the private implementation data for the LoggerFile class.
     */
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

    void LoggerFile::logInternal(const std::string& name, const std::string& data) {
        if (auto ts = getTimestep()) {
            std::stringstream ss;
            ss << "[" << *ts << "]; " << data;
            SPDLOG_LOGGER_INFO(data_->logger, "{}: {}", name, ss.str());
        } else {
            registerNormalLog();
            SPDLOG_LOGGER_INFO(data_->logger, "{}: {}", name, data);
        }
    }
}

#ifdef DBGNPROF_COMPILE_IN_ROS
#include <std_msgs/Float64.h>
#include <ros/message_traits.h>

namespace {
    // Whitelist of ROS 1 Noetic message types supported by rosbags parser
    // Reference: https://ternaris.gitlab.io/rosbags/topics/typesys-types-ros1_noetic.html
    // Extracted via automated curl + regex parsing (162 standard message types)
    static const std::unordered_set<std::string> ROSBAGS_SUPPORTED_TYPES = {
        "actionlib/TestAction",
        "actionlib/TestActionFeedback",
        "actionlib/TestActionGoal",
        "actionlib/TestActionResult",
        "actionlib/TestFeedback",
        "actionlib/TestGoal",
        "actionlib/TestRequestAction",
        "actionlib/TestRequestActionFeedback",
        "actionlib/TestRequestActionGoal",
        "actionlib/TestRequestActionResult",
        "actionlib/TestRequestFeedback",
        "actionlib/TestRequestGoal",
        "actionlib/TestRequestResult",
        "actionlib/TestResult",
        "actionlib/TwoIntsAction",
        "actionlib/TwoIntsActionFeedback",
        "actionlib/TwoIntsActionGoal",
        "actionlib/TwoIntsActionResult",
        "actionlib/TwoIntsFeedback",
        "actionlib/TwoIntsGoal",
        "actionlib/TwoIntsResult",
        "actionlib_msgs/GoalID",
        "actionlib_msgs/GoalStatus",
        "actionlib_msgs/GoalStatusArray",
        "bond/Constants",
        "bond/Status",
        "diagnostic_msgs/DiagnosticArray",
        "diagnostic_msgs/DiagnosticStatus",
        "diagnostic_msgs/KeyValue",
        "dynamic_reconfigure/BoolParameter",
        "dynamic_reconfigure/Config",
        "dynamic_reconfigure/ConfigDescription",
        "dynamic_reconfigure/DoubleParameter",
        "dynamic_reconfigure/Group",
        "dynamic_reconfigure/GroupState",
        "dynamic_reconfigure/IntParameter",
        "dynamic_reconfigure/ParamDescription",
        "dynamic_reconfigure/SensorLevels",
        "dynamic_reconfigure/StrParameter",
        "geometry_msgs/Accel",
        "geometry_msgs/AccelStamped",
        "geometry_msgs/AccelWithCovariance",
        "geometry_msgs/AccelWithCovarianceStamped",
        "geometry_msgs/Inertia",
        "geometry_msgs/InertiaStamped",
        "geometry_msgs/Point",
        "geometry_msgs/Point32",
        "geometry_msgs/PointStamped",
        "geometry_msgs/Polygon",
        "geometry_msgs/PolygonStamped",
        "geometry_msgs/Pose",
        "geometry_msgs/Pose2D",
        "geometry_msgs/PoseArray",
        "geometry_msgs/PoseStamped",
        "geometry_msgs/PoseWithCovariance",
        "geometry_msgs/PoseWithCovarianceStamped",
        "geometry_msgs/Quaternion",
        "geometry_msgs/QuaternionStamped",
        "geometry_msgs/Transform",
        "geometry_msgs/TransformStamped",
        "geometry_msgs/Twist",
        "geometry_msgs/TwistStamped",
        "geometry_msgs/TwistWithCovariance",
        "geometry_msgs/TwistWithCovarianceStamped",
        "geometry_msgs/Vector3",
        "geometry_msgs/Vector3Stamped",
        "geometry_msgs/Wrench",
        "geometry_msgs/WrenchStamped",
        "nav_msgs/GetMapAction",
        "nav_msgs/GetMapActionFeedback",
        "nav_msgs/GetMapActionGoal",
        "nav_msgs/GetMapActionResult",
        "nav_msgs/GetMapFeedback",
        "nav_msgs/GetMapGoal",
        "nav_msgs/GetMapResult",
        "nav_msgs/GridCells",
        "nav_msgs/MapMetaData",
        "nav_msgs/OccupancyGrid",
        "nav_msgs/Odometry",
        "nav_msgs/Path",
        "roscpp/Logger",
        "rosgraph_msgs/Clock",
        "rosgraph_msgs/Log",
        "rosgraph_msgs/TopicStatistics",
        "sensor_msgs/BatteryState",
        "sensor_msgs/CameraInfo",
        "sensor_msgs/ChannelFloat32",
        "sensor_msgs/CompressedImage",
        "sensor_msgs/FluidPressure",
        "sensor_msgs/Illuminance",
        "sensor_msgs/Image",
        "sensor_msgs/Imu",
        "sensor_msgs/JointState",
        "sensor_msgs/Joy",
        "sensor_msgs/JoyFeedback",
        "sensor_msgs/JoyFeedbackArray",
        "sensor_msgs/LaserEcho",
        "sensor_msgs/LaserScan",
        "sensor_msgs/MagneticField",
        "sensor_msgs/MultiDOFJointState",
        "sensor_msgs/MultiEchoLaserScan",
        "sensor_msgs/NavSatFix",
        "sensor_msgs/NavSatStatus",
        "sensor_msgs/PointCloud",
        "sensor_msgs/PointCloud2",
        "sensor_msgs/PointField",
        "sensor_msgs/Range",
        "sensor_msgs/RegionOfInterest",
        "sensor_msgs/RelativeHumidity",
        "sensor_msgs/Temperature",
        "sensor_msgs/TimeReference",
        "shape_msgs/Mesh",
        "shape_msgs/MeshTriangle",
        "shape_msgs/Plane",
        "shape_msgs/SolidPrimitive",
        "std_msgs/Bool",
        "std_msgs/Byte",
        "std_msgs/ByteMultiArray",
        "std_msgs/Char",
        "std_msgs/ColorRGBA",
        "std_msgs/Duration",
        "std_msgs/Empty",
        "std_msgs/Float32",
        "std_msgs/Float32MultiArray",
        "std_msgs/Float64",
        "std_msgs/Float64MultiArray",
        "std_msgs/Header",
        "std_msgs/Int16",
        "std_msgs/Int16MultiArray",
        "std_msgs/Int32",
        "std_msgs/Int32MultiArray",
        "std_msgs/Int64",
        "std_msgs/Int64MultiArray",
        "std_msgs/Int8",
        "std_msgs/Int8MultiArray",
        "std_msgs/MultiArrayDimension",
        "std_msgs/MultiArrayLayout",
        "std_msgs/String",
        "std_msgs/Time",
        "std_msgs/UInt16",
        "std_msgs/UInt16MultiArray",
        "std_msgs/UInt32",
        "std_msgs/UInt32MultiArray",
        "std_msgs/UInt64",
        "std_msgs/UInt64MultiArray",
        "std_msgs/UInt8",
        "std_msgs/UInt8MultiArray",
        "stereo_msgs/DisparityImage",
        "trajectory_msgs/JointTrajectory",
        "trajectory_msgs/JointTrajectoryPoint",
        "trajectory_msgs/MultiDOFJointTrajectory",
        "trajectory_msgs/MultiDOFJointTrajectoryPoint",
        "visualization_msgs/ImageMarker",
        "visualization_msgs/InteractiveMarker",
        "visualization_msgs/InteractiveMarkerControl",
        "visualization_msgs/InteractiveMarkerFeedback",
        "visualization_msgs/InteractiveMarkerInit",
        "visualization_msgs/InteractiveMarkerPose",
        "visualization_msgs/InteractiveMarkerUpdate",
        "visualization_msgs/Marker",
        "visualization_msgs/MarkerArray",
        "visualization_msgs/MenuEntry"
    };
}


namespace debug_and_profile_helper {
    void LoggerROS::dataDeleter::operator()(LoggerROS::data* p) {
        delete p;
    }

    LoggerROS::LoggerROS(const std::string& topicPrefix) : data_{ new data() } {
        // initialize the ROS node
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "debug_and_profile_helper");

        // initialize the pimplData object with a node handle with the provided topic prefix
        data_->nh = std::make_shared<ros::NodeHandle>();
        data_->topicPrefix = topicPrefix;
        data_->queue_size = 5000;
        data_->customFillFuncs_.clear();
    }

    void LoggerROS::log() const {
        static ros::Publisher pub = data_-> nh -> advertise<std_msgs::Float64>
                                    (data_->topicPrefix + "/empty_log", data_->queue_size);
        std_msgs::Float64 msg;
        msg.data = 0.0;
        pub.publish(msg);
    }
    
    bool LoggerROS::isRosbagsSupportedType(const std::string& type_name) {
        return ROSBAGS_SUPPORTED_TYPES.find(type_name) != ROSBAGS_SUPPORTED_TYPES.end();
    }
} // namespace debug_and_profile_helper
#endif // USE_ROS