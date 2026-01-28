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

#include <optional>
#include <stdexcept>

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

    /**
     * @brief Increase the timestep counter.
     *
     * This function initializes the timestep counter to 0 if it is not set, otherwise increases it by 1.
     * It also checks if any normal log has been issued before, if so, it will throw a runtime error.
     */
    void stepTimestep() {
        if (hasLoggedNormalData_) {
            throw std::runtime_error("LoggerBase: Cannot switch to Timestep mode after normal logging has occurred.");
        }
        if (!currentTimestep_.has_value()) {
            currentTimestep_ = 0.0;
        } else {
            currentTimestep_ = currentTimestep_.value() + 1.0;
        }
    }

    /**
     * @brief Get the current timestep.
     * 
     * @return std::optional<double> The current timestep.
     */
    std::optional<double> getTimestep() const {
        return currentTimestep_;
    }

protected:
    /**
     * @brief Register a normal log event.
     * 
     * This function marks that a normal log has occurred. It is providing checks if the logger is in Timestep mode.
     */
    void registerNormalLog() {
        // hasLoggedNormalData_ tracks if we have done normal logging.
        this->hasLoggedNormalData_ = true;
    }

    std::optional<double> currentTimestep_;
    bool hasLoggedNormalData_ = false;
};

} // namespace debug_and_profile_helper

#endif // DEBUG_AND_PROFILE_HELPER__LOGGER_BASE_HPP