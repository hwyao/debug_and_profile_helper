/**
 * @file 1-raw_LoggerFile_usage.cpp
 * @author Haowen Yao
 * @brief Example usage of the LoggerFile class.
 * @date 2024-12-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <Eigen/Dense>
#include <sstream>
#include <string>
#include <debug_and_profile_helper/helper_macros.hpp>

class some_class{
    public:
        some_class(int intValue, double doubleValue) : intValue(intValue), doubleValue(doubleValue) {}

        // implement the << operator for insertion of some_class objects
        friend std::ostream& operator<<(std::ostream& os, const some_class& obj){
            os << "class {\nintValue: " << obj.intValue << ", \ndoubleValue: " << obj.doubleValue << " }";
            return os;
        }

    private:
        int intValue;
        double doubleValue;
};

struct some_struct {
    int intValue;
    double doubleValue;
};

// Specialize the formatData function for the some_struct data type
template<>
std::string debug_and_profile_helper::LoggerFile::formatData<some_struct>(const some_struct& data) const{
    std::stringstream ss;
    ss << "struct {\nintValue: " << data.intValue << ", \ndoubleValue: " << data.doubleValue << " }";
    return ss.str();
}

int main() {
    auto& logger = debug_and_profile_helper::LoggerFile::getInstance();

    for (int i = 0; i < 10; ++i) {
        int intValue = i;
        double doubleValue = i * 0.1;
        Eigen::MatrixXd matrixValue = Eigen::MatrixXd::Random(3, 3);
        Eigen::Matrix2d matrix2dValue = Eigen::Matrix2d::Random();
        some_class someClassValue(i, i * 0.1);
        some_struct someStructValue = {i, i * 0.1};

        // The log function can support multiple data types by default if it supports the << operator
        // i.e. it can be used with std::cout << intValue
        logger.log("intValue", intValue);
        logger.log("doubleValue", doubleValue);
        logger.log("matrixValue", matrixValue);
        logger.log("matrix2dValue", matrix2dValue);

        // The log function can also support custom data types by specializing the formatData function
        // You should define the its << operator, or specialize the formatData function
        logger.log("someClassValue", someClassValue);
        logger.log("someStructValue", someStructValue);
    }

    return 0;
}