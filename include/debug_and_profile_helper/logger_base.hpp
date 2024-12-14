/**
 * @file logger_base.hpp
 * @author Haowen Yao
 * @brief Classes for logging messages, provides base class as singleton pattern.
 * @date 2024-12-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef DEBUG_AND_PROFILE_HELPER__LOGGER_BASE_HPP
#define DEBUG_AND_PROFILE_HELPER__LOGGER_BASE_HPP

#include <string>
#include <sstream>

namespace debug_and_profile_helper {

/**
 * @class LoggerBase
 * @brief A base class for logging messages.
 * 
 * This class is a template class that provides the base functionality for logging messages.
 */
template <typename T>
class LoggerBase {
public:
    /**
     * @brief Get the instance of the LoggerBase class.
     * 
     * @return T& The instance of the LoggerBase class.
     */
    static T& getInstance() {
        static T instance;
        return instance;
    }

    LoggerBase(const LoggerBase&) = delete;            /**< The copy constructor is deleted. */
    LoggerBase& operator=(const LoggerBase&) = delete; /**< The assignment operator is deleted. */

protected:
    LoggerBase() = default;          /**< The default constructor. */
    virtual ~LoggerBase() = default; /**< The virtual destructor. */

public:
    /**
     * @brief Log the data.
     * 
     * This is a dummy function that must be implemented in the derived classes, other polymorphic
     * functions can be added with the same name.
     */
    virtual void log() const = 0;
};

} // namespace debug_and_profile_helper

#endif // DEBUG_AND_PROFILE_HELPER__LOGGER_BASE_HPP