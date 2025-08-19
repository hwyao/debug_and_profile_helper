/**
 * @file helper_class.hpp
 * @author Haowen Yao
 * @brief Classes for logging messages to a file.
 * @date 2024-12-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef DEBUG_AND_PROFILE_HELPER__HELPER_CLASS_
#define DEBUG_AND_PROFILE_HELPER__HELPER_CLASS_

#include <memory>
#include <string>
#include <sstream>
#include <type_traits>

#include "debug_and_profile_helper/config.h"
#include "debug_and_profile_helper/logger_base.hpp"

namespace debug_and_profile_helper {
    /**
     * @class LoggerFile
     * @brief A class for logging messages to a file.
     * 
     * This class inherits from LoggerBase and provides functionality to log messages
     * to a file. 
     */
    class LoggerFile : public LoggerBase<LoggerFile> {
        friend class LoggerBase<LoggerFile>;    // Declare the Base class as a friend to use its protected members.
    private:
        class pimplData;                        
        /// A deleter for the unique pointer to the private implementation data.
        struct pimplDataDeleter {               
            void operator()(pimplData* p);
        };                                                             
        std::unique_ptr<pimplData, pimplDataDeleter> data_; ///< A unique pointer to the private implementation data. 
        
        /**
         * @brief Constructs a LoggerFile object and opens the log file. Default file path is empty. (current directory)
         * 
         * The log file is created based on the current date and time, initialize
         * the private implementation data. This is private to ensure that the LoggerFile
         * class is a singleton.
         */
        LoggerFile(const std::string& filePath = "");

        /**
         * @brief Destroy the Logger File object
         * 
         * Default destructor for the LoggerFile class.
         */
        ~LoggerFile() noexcept override = default;    
        
    public:
        /**
         * @brief log a message to the file.
         * 
         * Log a default message to the file.
         */
        void log() const;

        /**
         * @brief log a message to the file.
         * 
         * @tparam T The type of the data to log.
         * @param name The name of the data.
         * @param data The data to be logged.
         * 
         * Log a message to file. This will try to formatize the second parameter with \ref formatData function. 
         */
        template <typename T>
        void log(const std::string& name, const T& data) const {
            std::string formattedData = formatData<T>(data);
            logInternal(name, formattedData);
        }

    private:
        /**
         * @brief Check if the type T has an stream operator. 
         * 
         * @tparam T The type to check. 
         * 
         * Default implementation, triggered when type T does not have stream operator.
         */
        template <typename T, typename = void>
        struct has_stream_operator : std::false_type {};

        /**
         * @brief Check if the type T has an stream operator.
         * 
         * @tparam T The type to check. 
         * 
         * Specialized implementation, triggered when type T has stream operator. 
         */
        template <typename T>
        struct has_stream_operator<
            T, 
            std::void_t<decltype(std::declval<std::ostream&>() << std::declval<T>())>
        > : std::true_type {};

        /**
         * @brief A template to format the data for \ref logInternal function.
         * 
         * @tparam T The type of the data to log.
         * @param data the data to be formatted.
         * @return std::string, the formatted string.
         * 
         * Enabled when then has_stream_operator<T>::value is true.
         * Format the data as a string using ss << data if the type T supports stream operator.
         */
        template <typename T>
        typename std::enable_if<has_stream_operator<T>::value, std::string>::type
        formatData(const T& data) const{
            std::stringstream ss;
            ss << data;
            return ss.str();
        }

        /**
         * @brief A template to handle the error message when the type T is not supported with << operator.
         * 
         * @tparam T The type of the data to log.
         * @param data The data to be formatted.
         * @return std::string, the formatted string - but this template will never return, just for SFINAE.
         * 
         * Substitution Failure Is Not An Error (SFINAE) skill, enabled to give a human-readable compile error 
         * to notice the user that the type T is not supported for log().
         */
        template <typename T>
        typename std::enable_if<!has_stream_operator<T>::value, std::string>::type
        formatData(const T& data) const {
            static_assert(has_stream_operator<T>::value, 
            "\n\n Type T is not supported for log(). Please do one of the options: \
            \n1. implement operator<< for T, std::ostream \
            \n2. specialize formatData(const T& data). \
            \nFor exact which T, please check the error message above [T = ...].\n");
            return {};
        }

        /**
         * @brief Internal logger function that writes to the file.
         * 
         * @param name The name of the data.
         * @param data The data to be logged.
         */
        void logInternal(const std::string& name, const std::string& data) const;
    };
} // namespace debug_and_profile_helper

#ifdef DBGNPROF_COMPILE_IN_ROS
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <functional>
#include <typeindex>
#include <unordered_map>

namespace debug_and_profile_helper {
    /**
     * @class LoggerROS
     * @brief A class for logging messages to ROS.
     * 
     * This class inherits from LoggerBase and provides functionality to log messages to ROS. 
     */
    class LoggerROS : public LoggerBase<LoggerROS> {
        friend class LoggerBase<LoggerROS>;    ///< Declare the Base class as a friend to use its protected members. 
    private:
        /// A class to store the data. This is not private (PIMPL) since we have to match type for ROS message in header.
        class data {
        public:
            std::shared_ptr<ros::NodeHandle> nh;
            std::string topicPrefix;
            unsigned queue_size;
            std::unordered_map<std::type_index, std::function<void(void*, const void*)>> customFillFuncs_; 
        };
        /// A deleter for the unique pointer to the data.                       
        struct dataDeleter {                  
            void operator()(data* p);
        };                                     
        std::unique_ptr<data, dataDeleter> data_;    ///< A unique pointer to the data. 

        /**
         * @brief Constructs a LoggerROS object and initializes the ROS node handle. 
         * 
         * This is private to ensure that the LoggerROS class is a singleton.
         */
        LoggerROS(const std::string& topicPrefix = "debug_and_profile_helper");

    private:
        /**
         * @brief A template to get the ROS message type for the data type.
         * 
         * @tparam T The type of the data to log.
         * @tparam Enable A template parameter to enable SFINAE.
         * 
         * Default implementation, which is triggered when the type T is not supported, the matching
         * message type is void. 
         */
        template <typename T, typename Enable = void>
        struct ROSMessageType{
            using type = void;
        };

        /**
         * @brief Match std_msgs::Int32 for integral type T that equal or smaller than int.
         */
        template <typename T>
        struct ROSMessageType<T, typename std::enable_if<std::is_integral<T>::value && (sizeof(T) <= sizeof(int)) && std::is_signed<T>::value>::type> {
            using type = std_msgs::Int32;
        };

        /**
         * @brief Match std_msgs::UInt32 for unsigned integral type T that equal or smaller than unsigned int.
         */
        template <typename T>
        struct ROSMessageType<T, typename std::enable_if<std::is_integral<T>::value && (sizeof(T) <= sizeof(unsigned int)) && std::is_unsigned<T>::value>::type> {
            using type = std_msgs::UInt32;
        };

        /**
         * @brief Match std_msgs::Int64 for integral type T that larger than int.
         */
        template <typename T>
        struct ROSMessageType<T, typename std::enable_if<std::is_integral<T>::value && (sizeof(T) > sizeof(int)) && std::is_signed<T>::value>::type> {
            using type = std_msgs::Int64;
        };

        /**
         * @brief Match std_msgs::UInt64 for unsigned integral type T that larger than unsigned int.
         */
        template <typename T>
        struct ROSMessageType<T, typename std::enable_if<std::is_integral<T>::value && (sizeof(T) > sizeof(unsigned int)) && std::is_unsigned<T>::value>::type> {
            using type = std_msgs::UInt64;
        };

        /**
         * @brief Match std_msgs::Float64 for floating point type T.
         */
        template <typename T>
        struct ROSMessageType<T, typename std::enable_if<std::is_floating_point<T>::value>::type> {
            using type = std_msgs::Float64;
        };

        /**
         * @brief Match std_msgs::Float64MultiArray for all Eigen types (Matrix and expressions).
         * 
         * This specialization catches all types that inherit from Eigen::MatrixBase,
         * including both explicit matrices and expressions like Eigen::Product, Eigen::Sum, etc.
         */
        template <typename T>
        struct ROSMessageType<T, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<T>, T>::value>::type> {
            using type = std_msgs::Float64MultiArray;
        };

        /**
         * @brief A helper template to check if a conversion from numerical type T to ROS message type is supported.
         * 
         * @tparam T The type to check.
         * @tparam Enable The template parameter to enable SFINAE.
         * 
         * Default implementation, which is triggered when the type T has no template ROSMessageType specialization.
         * (with void type)
         */
        template <typename T, typename Enable = void>
        struct is_supported_type : std::false_type {};

        /**
         * @brief A helper template to check if a conversion from numerical type T to ROS message type is supported.
         * 
         * @tparam T The type to check.
         * @tparam Enable The template parameter to enable SFINAE.
         * 
         * Specialized implementation, which is triggered when the type T has a template ROSMessageType specialization.
         */
        template <typename T>
        struct is_supported_type<T, typename std::enable_if<!std::is_same<typename ROSMessageType<T>::type, void>::value>::type> : std::true_type {};  

        /**
         * @brief A helper template to check if a type T is an Eigen type (Matrix or expression).
         * 
         * @tparam T The type to check.
         * @tparam Enable SFINAE parameter.
         * 
         * Default implementation, is enabled to false when T is not an Eigen type.
         */
        template <typename T, typename Enable = void>
        struct is_eigen_matrix : std::false_type {};

        /**
         * @brief Specialization to detect any type derived from Eigen::MatrixBase.
         * 
         * @tparam T The type to check.
         * 
         * This catches all Eigen types including:
         * - Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>
         * - Eigen expressions like Eigen::Product, Eigen::Sum, etc.
         */
        template <typename T>
        struct is_eigen_matrix<T, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<T>, T>::value>::type> : std::true_type {};
      
        /**
         * @brief A template to fill the ROS message with the data.
         * 
         * @tparam MsgType The type of the ROS message.
         * @tparam DataType The type of the data.
         * @param msg The ROS message to be filled.
         * @param data The data to fill the ROS message.
         * 
         * Default implementation, which is triggered when the type DataType is integral or floating point type, 
         * the corresponding ROS message only has a simple data field.
         */
        template <typename MsgType, typename DataType>
        typename std::enable_if<std::is_integral<DataType>::value || std::is_floating_point<DataType>::value>::type
        fillMessage(MsgType& msg, const DataType& data) const{
            msg.data = data;
        }

        /**
         * @brief A template to fill the ROS message with Eigen data (Matrix or expression).
         * 
         * @tparam T The type of the Eigen data (Matrix or expression).
         * @param msg The ROS Float64MultiArray message to be filled.
         * @param data The Eigen data to fill the ROS message.
         * 
         * This handles all Eigen types by automatically calling .eval() when necessary.
         * For Matrix types, .eval() returns a reference to the same object (no copy).
         * For expression types, .eval() computes the result as a temporary matrix.
         */
        template <typename T>
        typename std::enable_if<std::is_base_of<Eigen::MatrixBase<T>, T>::value>::type
        fillMessage(std_msgs::Float64MultiArray& msg, const T& data) const {
            // .eval() works for both matrices and expressions
            // For matrices: returns a const reference (no copy)
            // For expressions: evaluates to a temporary matrix
            auto evaluated = data.eval();
            msg.layout.dim.resize(2);
            msg.layout.dim[0].label = "rows";
            msg.layout.dim[0].size = evaluated.rows();
            msg.layout.dim[0].stride = evaluated.rows() * evaluated.cols();
            msg.layout.dim[1].label = "cols";
            msg.layout.dim[1].size = evaluated.cols();
            msg.layout.dim[1].stride = evaluated.cols();
            msg.data.resize(evaluated.size());
            std::copy(evaluated.data(), evaluated.data() + evaluated.size(), msg.data.begin());
        }
        
    public:
        /**
         * @brief log a message to the file.
         * 
         * Log a message to the file with default message.
         */
        void log() const;

        /**
         * @brief log a message to the file.
         * 
         * @tparam T The type of the data to log.
         * @param pub_ptr The shared pointer to the ROS publisher, initialized if empty.
         * @param name The name of the data.
         * @param data The data to be logged.
         * @return void.
         * 
         * Log a message to ROS. This will use the ROSMessageType to get corresponding ROS message type,
         * Then fill the ROS message with either fillMessage() or custom fill function. pub_ptr is better
         * being initialized as static with nullptr outside in its own scope, so that publisher only 
         * initialized once and reused for multiple log calls.
         */
        template <typename T>
        typename std::enable_if<is_supported_type<T>::value>::type
        log(std::shared_ptr<ros::Publisher>& pub_ptr, const std::string& name, const T& data) {
            using MsgType = typename ROSMessageType<T>::type;
            if (!pub_ptr) {
                pub_ptr = std::make_shared<ros::Publisher>(data_ -> nh -> advertise<MsgType>
                                (data_->topicPrefix + "/" + name, data_->queue_size));
            }
            MsgType msg;

            if constexpr (std::is_integral<T>::value || std::is_floating_point<T>::value || is_eigen_matrix<T>::value) {
                // only enable compiling fillMessage when it is internally supported.
                this->fillMessage(msg, data);
            } else { 
                // otherwise use custom fill function
                auto it = data_ -> customFillFuncs_.find(std::type_index(typeid(T)));
                if (it != data_ -> customFillFuncs_.end()) {
                    it->second(&msg, &data);
                } else {
                    ROS_ERROR_STREAM_ONCE("debug_and_profile_helper: Type conversion template detected but function registered for synthesizing ROS message [" 
                                           + std::string(typeid(MsgType).name()) + "] from type [" + std::string(typeid(T).name())
                                           + "].\nPlease register a conversion function with LoggerROS::registerCustomType(fillFunc) with signature:\
                                             \n      void fillFunc([ROS_type]& msg, const [my_type]& data) { /* your way to fill the msg with data. */ }");
                }
            }
            
            pub_ptr -> publish(msg);
        }
    
        /**
         * @brief log a message to the file.
         * 
         * @tparam T The type of the data to log.
         * @param pub_ptr The shared pointer to the ROS publisher, initialized if empty.
         * @param name The name of the data.
         * @param data The data to be logged.
         * @return void.
         * 
         * Log a message to ROS. This will give a human-readable compile error to notice the user that the type T is not supported for log(),
         * indicated by is_supported_type<T>::value is false.
         */
        template <typename T>
        typename std::enable_if<!is_supported_type<T>::value>::type
        log(std::shared_ptr<ros::Publisher>& pub_ptr, const std::string& name, const T& data) {
            static_assert(is_supported_type<T>::value,
                "\n\nType T is not supported for log() with Publisher. Please do both (reference for helper_class.hpp for coding):\
                \n1. Set up a template to tell which ROS type that your type [T = my_type] can be converted to. \
                \n    template <typename T> \
                \n    struct LoggerROS::ROSMessageType<T, typename std::enable_if<std::is_same<T, [my_type]>::value> { using type = [ROS_type]; }\
                \n2. Set up a fill function for your type [T = my_type]. and register it with LoggerROS::registerCustomType(fillFunc). with signature:\
                \n    void fillFunc([ROS_type]& msg, const [my_type]& data) { /* your way to fill the msg with data. */ }\
                \nFor exact which T, please check the error message above [T = ...].\n");
            static_assert(!std::is_integral<T>::value, "This template is enabled, but T is a integral type, Should not reach here, contact the developer.");
            static_assert(!std::is_floating_point<T>::value, "This template is enabled, but T is a floating point type, Should not reach here, contact the developer.");
            static_assert(!is_eigen_matrix<T>::value, "This template is enabled, but T is a Eigen::Matrix type, Should not reach here, contact the developer."); 
        }

        /**
         * @brief Register a custom fill function for a custom data type.
         * 
         * @tparam MsgType The type of the ROS message.
         * @tparam DataType The type of the data.
         * @param fillFunc The function to fill the ROS message with the data.
         * 
         * Register a custom fill function for a custom data type. The fill function should have the signature:
         *    void fillFunc(MsgType& msg, const DataType& data) {  ... your way to fill the msg with data.  }
         */
        template <typename MsgType, typename DataType>
        void registerCustomType(std::function<void(MsgType&, const DataType&)> fillFunc) {
            data_ -> customFillFuncs_[std::type_index(typeid(DataType))] = [fillFunc](void* msg, const void* data) {
                fillFunc(*static_cast<MsgType*>(msg), *static_cast<const DataType*>(data));
            };
        }
    };
} // namespace debug_and_profile_helper
#endif // USE_ROS

#endif // DEBUG_AND_PROFILE_HELPER__HELPER_CLASS_