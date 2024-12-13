/**
 * @file 2-raw_LoggerROS_usage.cpp
 * @author Haowen Yao
 * @brief Example usage of the LoggerROS class.
 * @date 2024-12-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <chrono>
#include <thread>
#include <memory>
#include <debug_and_profile_helper/helper_macros.hpp>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <std_msgs/Int32.h>

class some_type {
public:
    int a;
    bool b;
};

// Specialize the ROSMessageType struct for the some_type data type
template <typename T>                 
struct debug_and_profile_helper::LoggerROS::ROSMessageType<T, typename std::enable_if<std::is_same<T, some_type>::value>::type> { using type = std_msgs::Int32; };

// Specialize the fillMessage function for the some_type data type filling into std_msgs::Int32
void myFillFunc(std_msgs::Int32& msg, const some_type& data) {
    msg.data = data.a;
}

int main(){
    auto& logger = debug_and_profile_helper::LoggerROS::getInstance();
    // Register the fill function for the some_type data type
    logger.registerCustomType(std::function<void(std_msgs::Int32&, const some_type&)>(myFillFunc));

    for (int i = 0; i < 50; i++) {
        // sleep 200ms
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // default log function
        logger.log();

        // Some types are internally supported by the logger, so that the publisher can be created automatically
        // This includes integer, floating point, Eigen::Matrix.
        // The use will be similar to a simple ROS publisher other than a complex publisher setup. Also user can fill
        // in a evaluation instead of forcing user to make a variable as reference.
        // This is the raw usage, the better way is to use the helper macros.
        static std::shared_ptr<ros::Publisher> pub_ptr1 = nullptr;
        logger.log(pub_ptr1, "test_int", i);

        static std::shared_ptr<ros::Publisher> pub_ptr2 = nullptr;
        logger.log(pub_ptr2, "test_double", i*1.0);

        static std::shared_ptr<ros::Publisher> pub_ptr3 = nullptr;
        logger.log(pub_ptr3, "test_double2", i*2.0);

        Eigen::Matrix<double, 3, 3> mat = Eigen::Matrix<double, 3, 3>::Random();
        static std::shared_ptr<ros::Publisher> pub_ptr4 = nullptr;
        logger.log(pub_ptr4, "test_eigen", mat);

        Eigen::Matrix<double, 2, 2> mat2 = Eigen::Matrix<double, 2, 2>::Random();
        static std::shared_ptr<ros::Publisher> pub_ptr5 = nullptr;
        logger.log(pub_ptr5, "test_eigen2", mat2);

        // For custom data types, the user needs to specialize the ROSMessageType struct and fillMessage function,
        // and register the fill function. Then it can be used like the default types.
        some_type st;
        st.a = i*5;
        st.b = i % 2 == 0;
        static std::shared_ptr<ros::Publisher> pub_ptr6 = nullptr;
        logger.log(pub_ptr6, "test_struct", st);
    }
}