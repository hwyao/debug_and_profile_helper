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
            static_assert(has_insertion_operator<T>::value, "\n\n Type T is not supported for log(). Please: \
            \n1. implement operator<< for T, std::ostream \
            \n2. or specialize formatData(const T& data). \
            \nFor exact which T, please check the error message above [T = ...].");
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
        struct pimplData;                       /**< Forward declaration of the private implementation data struct. */
        struct pimplDataDeleter {               /**< A deleter for the unique pointer to the private implementation data. */
            void operator()(pimplData* p);
        };
        std::unique_ptr<pimplData, pimplDataDeleter> data_;    /**< A unique pointer to the private implementation data. */
        
        /**
         * @brief Constructs a LoggerROS object and initializes the ROS node handle. 
         * 
         * This is private to ensure that the LoggerROS class is a singleton.
         */
        LoggerROS(const std::string& topicPrefix = "debug_and_profile_helper");

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
         * @param name The name of the data.
         * @param data The data to be logged.
         */
        template <typename T>
        void log(const std::string& name, const T& data) const{
            
        }
    };
} // namespace debug_and_profile_helper
#endif // USE_ROS

#endif // DEBUG_AND_PROFILE_HELPER__HELPER_CLASS_