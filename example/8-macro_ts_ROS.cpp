/**
 * @file 8-macro_ts_ROS.cpp
 * @author Haowen Yao
 * @brief Example usage of macros with ROS logger in timestamp mode with custom types
 * @date 2026-1-26
 * 
 * This example demonstrates the key bug fix:
 * 1. Built-in types with timestamp: packed into Float64MultiArray
 * 2. Custom type WITH header: timestamp injected into header.stamp  
 * 3. Custom type WITHOUT header: warning issued, message published without timestamp
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#define DBGNPROF_ENABLE_DEBUG
#define DBGNPROF_USE_ROS

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include "debug_and_profile_helper/helper_macros.hpp"
#include <Eigen/Dense>
#include <thread>
#include <chrono>

// Custom type that will be mapped to a ROS message WITH header
struct VehicleState {
    double x, y, velocity;
};

// Specialize ROSMessageType for VehicleState -> geometry_msgs::PoseStamped (has header)
template <typename T>
struct debug_and_profile_helper::LoggerROS::ROSMessageType<T, typename std::enable_if<std::is_same<T, VehicleState>::value>::type> { 
    using type = geometry_msgs::PoseStamped; 
};

// Fill function for VehicleState -> PoseStamped
void fillVehicleState(geometry_msgs::PoseStamped& msg, const VehicleState& data) {
    msg.header.frame_id = "world";
    // Timestamp will be automatically injected by the logger!
    msg.pose.position.x = data.x;
    msg.pose.position.y = data.y;
    msg.pose.position.z = data.velocity;
}

// Custom type that will be mapped to a ROS message WITHOUT header
struct MotorCommand {
    int motor_id;
    int pwm_value;
};

// Specialize ROSMessageType for MotorCommand -> std_msgs::Int32 (NO header)
template <typename T>
struct debug_and_profile_helper::LoggerROS::ROSMessageType<T, typename std::enable_if<std::is_same<T, MotorCommand>::value>::type> { 
    using type = std_msgs::Int32; 
};

// Fill function for M otorCommand -> Int32
void fillMotorCommand(std_msgs::Int32& msg, const MotorCommand& data) {
    msg.data = data.motor_id * 1000 + data.pwm_value;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "timestamp_ROS_custom_types_example");
    ros::NodeHandle nh;

    // Register custom type fill functions
    auto& logger = debug_and_profile_helper::LoggerROS::getInstance();
    logger.registerCustomType(std::function<void(geometry_msgs::PoseStamped&, const VehicleState&)>(fillVehicleState));
    logger.registerCustomType(std::function<void(std_msgs::Int32&, const MotorCommand&)>(fillMotorCommand));

    for (int i = 0; i < 100; ++i) {
        // Increment timestamp
        DBGNPROF_COUNT_TIMESTEP();
        
        // 1. Built-in types: timestamp packed into Float64MultiArray format
        int int_val = i * 10;
        DBGNPROF_LOG("int_val", int_val);
        
        double float_val = i * 1.1;
        DBGNPROF_LOG("float_val", float_val);

        Eigen::Matrix<double, 3, 3> mat = Eigen::Matrix<double, 3, 3>::Random();
        DBGNPROF_LOG("matrix", mat);
        
        // 2. Custom type WITH header: timestamp injected into msg.header.stamp
        // This demonstrates the bug fix! The timestamp goes into the header, not packed into array
        VehicleState vehicle{i * 0.5, i * 0.3, i * 0.1};
        DBGNPROF_LOG("vehicle_state", vehicle);
        
        // 3. Custom type WITHOUT header: warning will be issued (throttled to every 5 seconds)
        // The message will still be published, just without timestamp
        MotorCommand motor{1, 128 + i % 128};
        DBGNPROF_LOG("motor_command", motor);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ros::spinOnce();
    }
}
