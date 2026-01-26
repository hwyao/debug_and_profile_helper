/**
 * @file 3-raw_ts_File.cpp
 * @author Haowen Yao
 * @brief Example of raw LoggerFile usage with timestamp mode and custom types
 * @date 2026-1-26
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include <chrono>
#include <thread>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <debug_and_profile_helper/helper_class.hpp>

// Custom type demonstrating stream operator
struct SensorReading {
    int sensor_id;
    double temperature;
    double humidity;
    
    // Implement << operator for file logging
    friend std::ostream& operator<<(std::ostream& os, const SensorReading& reading) {
        os << "Sensor[" << reading.sensor_id << "]: temp=" << reading.temperature 
           << "°C, humidity=" << reading.humidity << "%";
        return os;
    }
};

// Another custom type using specialized formatData
struct DataPoint {
    double x, y, z;
};

// Specialize formatData for DataPoint
template<>
std::string debug_and_profile_helper::LoggerFile::formatData<DataPoint>(const DataPoint& data) const {
    std::stringstream ss;
    ss << "Point(" << data.x << ", " << data.y << ", " << data.z << ")";
    return ss.str();
}

int main() {
    auto& logger = debug_and_profile_helper::LoggerFile::getInstance();
    
    for (int i = 0; i < 10; ++i) {
        // Increment timestep counter
        logger.stepTimestep();
        
        // Log built-in types with timestamp
        logger.log("int_val", i * 10);
        logger.log("float_val", i * 1.1);
        
        Eigen::Matrix<double, 3, 3> mat = Eigen::Matrix<double, 3, 3>::Random();
        logger.log("matrix", mat);
        
        // Log custom types with timestamp using stream operator
        SensorReading sensor{i % 5, 20.0 + i * 0.5, 45.0 + i * 2.0};
        logger.log("sensor_reading", sensor);
        
        // Log custom types with timestamp using specialized formatData
        DataPoint point{i * 0.1, i * 0.2, i * 0.3};
        logger.log("data_point", point);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return 0;
}
