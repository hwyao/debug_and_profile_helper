/**
 * @file 6-timestamp_ROS.cpp
 * @author Haowen Yao
 * @brief Example usage of the macros
 * @date 2026-1-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#define DBGNPROF_ENABLE_DEBUG
#define DBGNPROF_USE_ROS

#include <ros/ros.h>
#include "debug_and_profile_helper/helper_macros.hpp"
#include <Eigen/Dense>
#include <thread>
#include <chrono>

int main(int argc, char** argv) {
    ros::init(argc, argv, "timestamp_ROS_example");
    ros::NodeHandle nh;

    for (int i = 0; i < 100; ++i) {
        DBGNPROF_COUNT_TIMESTEP();
        
        int int_val = i * 10;
        DBGNPROF_LOG("int_val", int_val);
        
        double float_val = i * 1.1;
        DBGNPROF_LOG("float_val", float_val);

        Eigen::Matrix<double, 3, 3> mat = Eigen::Matrix<double, 3, 3>::Random();
        DBGNPROF_LOG("matrix", mat);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ros::spinOnce();
    }
}
