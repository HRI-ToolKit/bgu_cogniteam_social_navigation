cmake_minimum_required(VERSION 3.5)
project(social_navigation_manager)

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

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

find_package(pcl_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# find_package(nav_2d_utils REQUIRED)
#find_package(nav_2d_msgs REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  /opt/ros/foxy/include
)

set(dependencies
  angles
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  nav2_msgs
  nav2_costmap_2d
  # nav_2d_utils
  #nav_2d_msgs
  nav2_util
  nav2_core
  pluginlib
  sensor_msgs
  pcl_ros
)

add_executable(social_navigation_manager_node src/social_navigation_manager_node.cpp)
ament_target_dependencies(social_navigation_manager_node 
    angles
    rclcpp
    rclcpp_action
    rclcpp_components
    std_msgs
    nav2_msgs
    nav2_costmap_2d
    # nav_2d_utils
    #nav_2d_msgs
    nav2_util
    nav2_core
    pluginlib

    geometry_msgs  # maybe don't need this
    
    rcl_interfaces
    rcutils
    sensor_msgs
    tf2_geometry_msgs
  )


include_directories()
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(social_navigation_manager_node
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_include_directories(social_navigation_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS social_navigation_manager_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()
