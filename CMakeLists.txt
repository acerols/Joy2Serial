cmake_minimum_required(VERSION 3.5)
project(joy2serial)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

find_package(agent_msgs REQUIRED)
find_package(okarobo_msgs REQUIRED)


add_executable(joy2serial
    src/joystick_node.cpp
    src/joy2serial.cpp
)

target_link_libraries(joy2serial
    ${Boost_LIBRARIES}
    Threads::Threads
)

ament_target_dependencies(joy2serial
    rclcpp
    sensor_msgs
    Boost
    Threads
)

target_include_directories(joy2serial
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(robo2ros
  src/robo2ros/robo_sub_pub_node.cpp
  src/robo2ros/robo_sub_pub.cpp
  src/serial.cpp
)

ament_target_dependencies(robo2ros
  rclcpp
  sensor_msgs
  agent_msgs
  okarobo_msgs
)

target_include_directories(robo2ros
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
    joy2serial
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  robo2ros
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
