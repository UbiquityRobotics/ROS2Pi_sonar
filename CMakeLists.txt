cmake_minimum_required(VERSION 3.5)
project(pi_sonar)

set(CMAKE_CXX_STANDARD 17) # use C++14

# Compiler flags for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(sensor_msgs REQUIRED)

# Locate pigpio library
find_library(PIGPIO_LIBRARY pigpio PATHS /usr/local/lib)
find_path(PIGPIO_INCLUDE_DIR pigpio.h PATHS /usr/local/include)

if (NOT PIGPIO_LIBRARY OR NOT PIGPIO_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find pigpio library or headers")
endif()

# Create the executable
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch*")
  add_executable(pi_sonar src/pi_sonar.cpp)
  target_compile_options(pi_sonar PRIVATE -std=c++17)

  # Link libraries
  target_link_libraries(pi_sonar
    pigpiod_if2
    pthread
  )

  # Install the executable
  install(TARGETS pi_sonar
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Link the ROS 2 libraries to the executable using ament_target_dependencies
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "sensor_msgs"
  "diagnostic_updater"
)

# Install the executable
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Package information
ament_package()
