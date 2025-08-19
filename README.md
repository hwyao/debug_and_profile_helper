# Debug and Profile Helper

A lightweight ROS package for debugging and profiling C++ applications. Provides flexible logging capabilities and stopwatch-style timing measurements that can be easily toggled on/off through compile-time macros.

![README title page](image/README.drawio.svg)

## 🚀 Features

- **File and ROS Logging**: Support both file-based and ROS topic-based logging.
- **Stopwatch Profiling**: Built-in timing measurements for performance analysis.
- **Compile-time Control**: Enable/disable features through CMake options and preprocessor macros.
- **Flexible Logging**: Log various data types including primitives, Eigen matrices, and custom types with easy integration.

## ⬇️ Installation

### Standalone Usage (CMake)

Use this CMake library with `FetchContent()`, then link `debug_and_profile_helper` target to your code.

```cmake
# Import this library
include(FetchContent)
FetchContent_Declare(
    debug_and_profile_helper
    GIT_REPOSITORY https://github.com/hwyao/debug_and_profile_helper
    GIT_TAG main
)
FetchContent_MakeAvailable(debug_and_profile_helper)

# Use this library
target_link_libraries(main PRIVATE debug_and_profile_helper)
```

### ROS Usage

Clone this package into your ROS workspace's `src` directory.

```bash
cd ~/<your_workspace>/src
git clone https://github.com/hwyao/debug_and_profile_helper
```

For other package that would like to use this package you should configure its `CMakeLists.txt` like this:
```cmake
# find debug_and_profile_helper package and components
find_package(catkin REQUIRED COMPONENTS
  # ... others
  debug_and_profile_helper
)

# ... other cmake configuration

# link and include catkin for the executable target that need this.
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(main_node src/main.cpp)
target_link_libraries(main_node
  ${catkin_LIBRARIES}
)

# add C++ 17 compile support for executable target(!)
target_compile_features(main_node PUBLIC 
  cxx_std_17
)
```

Then build it with `catkin build`.


## ▶️ Quick Start

### Using Convenient Macros

It is quite straightforward and self-explanatory to see the code and understand its functionality:

```cpp
// Configure the helper behavior
#define DBGNPROF_USE_FILE           // Use file logging
//#define DBGNPROF_USE_ROS          // Use ROS logging
#define DBGNPROF_ENABLE_DEBUG       // Enable debug logging
#define DBGNPROF_ENABLE_PROFILE     // Enable profiling
#include <debug_and_profile_helper/helper_macros.hpp>

int main() {
    // Debug logging
    DBGNPROF_LOG("value", 42);
    DBGNPROF_LOG("matrix", Eigen::Matrix2d::Random());
    
    // Performance profiling
    DBGNPROF_START_CLOCK;
    // ... your code here ...
    DBGNPROF_STOP_CLOCK("operation_name");
    
    return 0;
}
```

You can control the library behavior through these preprocessor definitions:

- `DBGNPROF_USE_FILE`: Enable file-based logging.
- `DBGNPROF_USE_ROS`: Enable ROS topic-based logging (overwriting `DBGNPROF_USE_FILE` if both exists)
- `DBGNPROF_ENABLE_DEBUG`: Enable debug logging macros.
- `DBGNPROF_ENABLE_PROFILE`: Enable profiling macros.

These debug logging macros are enabled by defining `DBGNPROF_ENABLE_DEBUG`:
- `DBGNPROF_LOG("name", value);`: Log the value with the given name. Output target according to `DBGNPROF_USE_FILE` and `DBGNPROF_USE_ROS`. 

These profile logging macros are enabled by defining `DBGNPROF_ENABLE_PROFILE`:
- `DBGNPROF_START_CLOCK;`: Start timing.
- `DBGNPROF_STOP_CLOCK("clock_name");`: Stop timing and output result in nanosecond to the clock name.

There are several extra macros like `DBGNPROF_LOG_TO_FILE`. See [include/debug_and_profile_helper/helper_macros.hpp](https://github.com/hwyao/debug_and_profile_helper/blob/main/include/debug_and_profile_helper/helper_macros.hpp) for more information.

>Remember several limitations when you use the macro:
>- You cannot write another `DBGNPROF_START_CLOCK` next to another itself without `DBGNPROF_STOP_CLOCK` inbetween.
>- You cannot write `DBGNPROF_LOG` inside performance profiling blocking since logging also takes time.
>- Due to macro expansion limit, currently this macro is unable to be executed inside the loop with dependency of loop index, e.g.
>    ```cpp
>    for(int i=1;i<=10;i++){
>        DBGNPROF_LOG("value"+std::to_string(i), i);
>    }
>    ```

### Using header files

You can wee [1-raw_LoggerFile.cpp](https://github.com/hwyao/debug_and_profile_helper/blob/main/example/1-raw_LoggerFile.cpp) and [2-raw_LoggerROS.cpp](https://github.com/hwyao/debug_and_profile_helper/blob/main/example/2-raw_LoggerROS.cpp) for precise information about how to use the header files to log the result.

## Custom Data Types

Common types are supported for both loggers. You can integrate custom data types into the logging system.

The minimal supporting set for both loggers are:
- All primitive types: `int`, `double`, ...
- Eigen matrices and its evaluation: `Eigen::MatrixXd`, `mat1*mat2`, ...

### File Logger 

For file logger, according to its backend [spdlog](https://github.com/gabime/spdlog), all classes that supports stream operator is accepted. To extend the support for custom types, you can define stream operator or specialize `formatData` function.

#### Method 1: Define stream operator

Define a stream operator that outputs a string to `std::ostream` object.

```cpp
class MyClass {
    friend std::ostream& operator<<(std::ostream& os, const MyClass& obj) {
        os << "MyClass{value: " << obj.value << "}";
        return os;
    }
private:
    int value;
};
```

#### Method 2: Specialize formatData function

Specialize the `string formatData<T>(const T& data)` function for your custom type `T` and return a string as output.

```cpp
template<>
std::string debug_and_profile_helper::LoggerFile::formatData<MyStruct>(const MyStruct& data) const {
    return "MyStruct{" + std::to_string(data.value) + "}";
}
```

### ROS Logger

For ROS logger, the minimal supporting set mentioned above is implemented using a technique called [Substitution Failure Is Not An Error (SFINAE)](https://en.cppreference.com/w/cpp/language/sfinae.html). For custom type `some_type`, you need to specialize `ROSMessageType` to indicate which ROS message type `R_T` that `some_type` corresponds. And specialize `myFillFunc(R_T&, const some_type&)` to instruct how to fill in `R_T` with `some_type`.

#### Method: Specialize `ROSMessageType`, `myFillFunc` and register 

```cpp
// Specialize the ROSMessageType struct for the some_type
// Declare that std_msgs::Int32 is matching your custom type some_type.
template <typename T>                 
struct debug_and_profile_helper::LoggerROS::ROSMessageType<T, typename std::enable_if<std::is_same<T, some_type>::value>::type> { using type = std_msgs::Int32; };

// Specialize the fillMessage function for the some_type.
// Declare how to fill in ROS type std_msgs::Int32 with your custom type some_type.
void myFillFunc(std_msgs::Int32& msg, const some_type& data) {
    msg.data = data.a;
}

int main(){
    // ... get the ROS logger instance logger
    logger.registerCustomType(std::function<void(std_msgs::Int32&, const some_type&)>(myFillFunc));
}
```

## Examples

The package includes several examples in the `example/` directory:
- `1-raw_LoggerFile.cpp`: File logger example. With custom type example.
- `2-raw_LoggerROS.cpp`: ROS-based logging example. With custom type example.
- `3-macro_File.cpp`: Macro-based usage with profiling using file output.
- `4-macro_ROS.cpp`: Macro-based usage with profiling using ROS output.

