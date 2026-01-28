/**
 * @file 4-raw_ts_ROS.cpp
 * @author Haowen Yao
 * @brief Example of raw LoggerROS usage with timestamp mode
 * @date 2026-1-26
 * 
 * This example demonstrates:
 * 1. Built-in types with timestamp (packed Float64MultiArray format)
 * 2. Custom type with header (timestamp injected into header.stamp)
 * 3. Custom type without header (warning issued)
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include <chrono>
#include <thread>
#include <memory>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <debug_and_profile_helper/helper_class.hpp>

// Custom type that will be mapped to a ROS message WITH header
struct RobotPose {
    double x, y, theta;
};

// Specialize ROSMessageType for RobotPose -> geometry_msgs::PoseStamped (has header)
template <typename T>
struct debug_and_profile_helper::LoggerROS::ROSMessageType<T, typename std::enable_if<std::is_same<T, RobotPose>::value>::type> { 
    using type = geometry_msgs::PoseStamped; 
};

// Fill function for RobotPose -> PoseStamped
void fillRobotPose(geometry_msgs::PoseStamped& msg, const RobotPose& data) {
    msg.header.frame_id = "map";
    msg.pose.position.x = data.x;
    msg.pose.position.y = data.y;
    msg.pose.orientation.z = std::sin(data.theta / 2.0);
    msg.pose.orientation.w = std::cos(data.theta / 2.0);
}

// Custom type that will be mapped to a ROS message WITHOUT header
struct SimpleCounter {
    int count;
};

// Specialize ROSMessageType for SimpleCounter -> std_msgs::Int32 (NO header)
template <typename T>
struct debug_and_profile_helper::LoggerROS::ROSMessageType<T, typename std::enable_if<std::is_same<T, SimpleCounter>::value>::type> { 
    using type = std_msgs::Int32; 
};

// Fill function for SimpleCounter -> Int32
void fillSimpleCounter(std_msgs::Int32& msg, const SimpleCounter& data) {
    msg.data = data.count;
}

int main() {
    auto& logger = debug_and_profile_helper::LoggerROS::getInstance();
    
    // Register custom type fill functions
    logger.registerCustomType(std::function<void(geometry_msgs::PoseStamped&, const RobotPose&)>(fillRobotPose));
    logger.registerCustomType(std::function<void(std_msgs::Int32&, const SimpleCounter&)>(fillSimpleCounter));
    
    for (int i = 0; i < 100; ++i) {
        // Increment timestep counter
        logger.stepTimestep();
        
        // 1. Built-in types: timestamp packed into Float64MultiArray
        static std::shared_ptr<ros::Publisher> pub_int = nullptr;
        logger.log(pub_int, "int_val", i * 10);
        
        static std::shared_ptr<ros::Publisher> pub_float = nullptr;
        logger.log(pub_float, "float_val", i * 1.1);
        
        Eigen::Matrix<double, 3, 3> mat = Eigen::Matrix<double, 3, 3>::Random();
        static std::shared_ptr<ros::Publisher> pub_mat = nullptr;
        logger.log(pub_mat, "matrix", mat);
        
        // 2. Custom type WITH header: timestamp injected into header.stamp
        RobotPose pose{i * 0.1, i * 0.05, i * 0.01};
        static std::shared_ptr<ros::Publisher> pub_pose = nullptr;
        logger.log(pub_pose, "robot_pose", pose);
        
        // 3. Custom type WITHOUT header: warning will be issued (throttled to once every 5s)
        SimpleCounter counter{i};
        static std::shared_ptr<ros::Publisher> pub_counter = nullptr;
        logger.log(pub_counter, "simple_counter", counter);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ros::spinOnce();
    }
    
    return 0;
}
