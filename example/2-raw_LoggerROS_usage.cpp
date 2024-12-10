#include <chrono>
#include <thread>
#include <debug_and_profile_helper/helper_macros.hpp>

int main(){
    auto& logger = debug_and_profile_helper::LoggerROS::getInstance();

    for (int i = 0; i < 100; ++i) {
        // sleep 200ms
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // call default log function
        logger.log();
    }
}