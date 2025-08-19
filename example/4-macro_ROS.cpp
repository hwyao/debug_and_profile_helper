/**
 * @file 3-macro_combine_standalone.cpp
 * @author Haowen Yao
 * @brief Example usage of the macros
 * @date 2024-12-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */

// These macros can be changed for manipulating the behavior of the helper macros
//#define DBGNPROF_USE_FILE             // Use file as sink to log data, also default aliases will be <>_TO_FILE version
#define DBGNPROF_USE_ROS                // Use ROS as sink to log data, also default aliases will be <>_TO_ROS version
// if both DBGNPROF_USE_FILE and this are defined, aliases will be prioritized to ROS version.
// if neither of them are defined, all macros will be empty implementation.
#define DBGNPROF_ENABLE_DEBUG           // Enable the debug logging, otherwise will be empty implementation
#define DBGNPROF_ENABLE_PROFILE         // Enable the profiling with clock, otherwise will be empty implementation

#include <chrono>
#include <thread>
#include <random>
#include <Eigen/Dense>
#include <debug_and_profile_helper/helper_macros.hpp>

int main() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(20, 200);
    
    for (int i = 0; i < 15; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Log some data
        DBGNPROF_LOG("intValue", i);
        DBGNPROF_LOG("doubleValue", i * 0.1);
        DBGNPROF_LOG("doubleValue2", i * 0.2);
        Eigen::Matrix<double, 3, 3> mat = Eigen::Matrix<double, 3, 3>::Random();
        DBGNPROF_LOG("matrix", mat);
        DBGNPROF_LOG("matrix2", Eigen::MatrixXd::Random(2, 2));

        // Do some profiling
        int random_number = dis(gen);
        DBGNPROF_LOG("randomNumber", random_number);

        DBGNPROF_START_CLOCK; 
        //DBGNPROF_START_CLOCK;                           // This will cause an error, because the clock is running, you cannot start it again.
        //DBGNPROF_LOG("randomNumber2", random_number);   // This will cause an error if clock is enabled, because the clock is running, your logging will take time.
        std::this_thread::sleep_for(std::chrono::milliseconds(random_number));
        DBGNPROF_STOP_CLOCK("randomSleep");

        DBGNPROF_START_CLOCK;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        DBGNPROF_STOP_CLOCK("fixedSleep");                // If you forget to stop the clock, the LOG at next iteration will throw error.

        DBGNPROF_LOG("dummy_separator", 0.0);
    }
}