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
        friend class LoggerBase<LoggerFile>;    /**< Declare the Base class as a friend to use its protected members. */
    private:
        class pimplData;                        /**< Forward declaration of the private implementation data struct. */
        struct pimplDataDeleter {               /**< A deleter for the unique pointer to the private implementation data. */
            void operator()(pimplData* p);
        };
        std::unique_ptr<pimplData, pimplDataDeleter> data_; /**< A unique pointer to the private implementation data. */
        
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
         * This function will log a message to the file, the message itself is a default message.
         */
        void log() const;

        /**
         * @brief log a message to the file.
         * 
         * @tparam T The type of the data to log.
         * @param name The name of the data.
         * @param data The data to be logged.
         * 
         * This template function will make second parameter more flexible, it will try to
         * process the second parameter as string with sstream ss << data. If this fails,
         * please directly call the log function with the second parameter as a formatted string,
         * or formulate formatData<typename T2>(const T2& data) with your own data type.
         */
        template <typename T>
        void log(const std::string& name, const T& data) const {
            std::string formattedData = formatData<T>(data);
            logInternal(name, formattedData);
        }

    private:
        /**
         * @def has_insertion_operator<T, void>
         * @brief Check if the type T has an insertion operator. 
         * 
         * @tparam T The type to check. This is a default implementation, which is triggered when
         * the type T does not have an insertion operator.
         */
        template <typename T, typename = void>
        struct has_insertion_operator : std::false_type {};

        /**
         * @def has_insertion_operator<T, std::void_t<decltype(std::declval<std::ostream&>() << std::declval<T>())>>
         * @brief Check if the type T has an insertion operator.
         * 
         * @tparam T The type to check. This is a specialized implementation, which is triggered when
         * the type T has an insertion operator. If the type T has an insertion operator, std::void_t
         * will be void, and the struct will be specialized to std::true_type (true). Otherwise, std::void_t 
         * will not be successfully instantiated, and \ref has_insertion_operator<T> will take over, so the
         * struct will be std::false_type (false).
         */
        template <typename T>
        struct has_insertion_operator<
            T, 
            std::void_t<decltype(std::declval<std::ostream&>() << std::declval<T>())>
        > : std::true_type {};

        /**
         * @def formatData(const T& data)
         * @brief format the data for \ref logInternal function.
         * 
         * @tparam T The type of the data to log.
         * @param data the data to be formatted.
         * @return std::enable_if<has_insertion_operator<T>::value, std::string>::type. Which is basically a string.
         * 
         * This template is only successfully instantiated when then has_insertion_operator<T>::value is true.
         * It will format the data as a string using std::stringstream ss << data if the type T supports insertion operator.
         */
        template <typename T>
        typename std::enable_if<has_insertion_operator<T>::value, std::string>::type
        formatData(const T& data) const{
            std::stringstream ss;
            ss << data;
            return ss.str();
        }

        /**
         * @brief A template to to handle the error message when the type T is not supported with << operator.
         * 
         * @tparam T The type of the data to log.
         * @param data The data to be formatted.
         * @return std::enable_if<!has_insertion_operator<T>::value, std::string>::type, which is basically a string.
         * 
         * This uses with Substitution Failure Is Not An Error (SFINAE) skill for the code, so that if the type T
         * does not support the insertion operator, the compiler will not throw too many error around the \ref formatData
         * function. Instead, here will throw a more human-readable error message to the user.
         */
        template <typename T>
        typename std::enable_if<!has_insertion_operator<T>::value, std::string>::type
        formatData(const T& data) const {
            static_assert(has_insertion_operator<T>::value, 
            "\n\n Type T is not supported for log(). Please do one of the options: \
            \n1. implement operator<< for T, std::ostream \
            \n2. specialize formatData(const T& data). \
            \nFor exact which T, please check the error message above [T = ...].\n");
            return {};
        }

        /**
         * @def logInternal(const std::string& name, const std::string& data) const
         * @brief Internal function to log the message to the file.
         * 
         * @param name The name of the data.
         * @param data The data to be logged.
         * 
         * This internal function for \ref LoggerFile class 
         */
        void logInternal(const std::string& name, const std::string& data) const;
    };
} // namespace debug_and_profile_helper

#ifdef USE_ROS
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
     * This class inherits from LoggerBase and provides functionality to log messages
     * to ROS. 
     */
    class LoggerROS : public LoggerBase<LoggerROS> {
        friend class LoggerBase<LoggerROS>;    /**< Declare the Base class as a friend to use its protected members. */
    private:        
        class data {
        public:
            std::shared_ptr<ros::NodeHandle> nh;
            std::string topicPrefix;
            unsigned queue_size;
            std::unordered_map<std::type_index, std::function<void(void*, const void*)>> customFillFuncs_; 
        };                                    /**< A class to store the data. This is not private since we have to match type for ROS message. */
        struct dataDeleter {                  /**< A deleter for the unique pointer to the data. */
            void operator()(data* p);
        };
        std::unique_ptr<data, dataDeleter> data_;    /**< A unique pointer to the data. */
        
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
         * @brief Match std_msgs::Float64MultiArray for Eigen::Matrix
         */
        template <typename T, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
        struct ROSMessageType<Eigen::Matrix<T, Rows, Cols, Options, MaxRows, MaxCols>, typename std::enable_if<std::is_floating_point<T>::value>::type> {
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
         * @brief A helper template to check if a type T is an Eigen::Matrix type.
         * 
         * @tparam T The type to check.
         * 
         * Default implementation, is enabled to false when T is not an Eigen::Matrix type.
         */
        template <typename T>
        struct is_eigen_matrix : std::false_type {};

        /**
         * @brief A helper template to check if a type T is an Eigen::Matrix type.
         * 
         * @tparam Scalar The scalar type of the Eigen::Matrix.
         * @tparam Rows The number of rows of the Eigen::Matrix.
         * @tparam Cols The number of columns of the Eigen::Matrix.
         * @tparam Options The options of the Eigen::Matrix.
         * @tparam MaxRows The maximum number of rows of the Eigen::Matrix.
         * @tparam MaxCols The maximum number of columns of the Eigen::Matrix.
         * 
         * Specialized implementation, is enabled to true when T is an Eigen::Matrix type.
         */
        template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
        struct is_eigen_matrix<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> : std::true_type {};
      
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
         * @brief A template to fill the ROS message with the data of Eigen::Matrix type.
         * 
         * @tparam Scalar The scalar type of the Eigen::Matrix.
         * @tparam Rows The number of rows of the Eigen::Matrix.
         * @tparam Cols The number of columns of the Eigen::Matrix.
         * @tparam Options The options of the Eigen::Matrix.
         * @tparam MaxRows The maximum number of rows of the Eigen::Matrix.
         * @tparam MaxCols The maximum number of columns of the Eigen::Matrix.
         * @param msg The ROS Float64MultiArray message to be filled.
         * @param data The Eigen::Matrix data to fill the ROS message.
         */
        template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
        void fillMessage(std_msgs::Float64MultiArray& msg, const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& data) const{
            msg.layout.dim.resize(2);
            msg.layout.dim[0].label = "rows";
            msg.layout.dim[0].size = data.rows();
            msg.layout.dim[0].stride = data.rows() * data.cols();
            msg.layout.dim[1].label = "cols";
            msg.layout.dim[1].size = data.cols();
            msg.layout.dim[1].stride = data.cols();
            msg.data.resize(data.size());
            std::copy(data.data(), data.data() + data.size(), msg.data.begin());
        }

    public:
        /**
         * @brief log a message to the file.
         * 
         * This function will log a message to the file, the message itself is a default message.
         */
        void log() const;

        /**
         * @brief log a message to ROS.
         * 
         * @tparam T The type of the data to log.
         * @param pub_ptr The shared pointer to the ROS publisher, initialized if empty.
         * @param name The name of the data.
         * @param data The data to be logged.
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