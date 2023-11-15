cmake_minimum_required(VERSION 3.5)
project(social_navigation_ui)

# Find dependencies
find_package(ament_cmake REQUIRED)



# Install executable
install(TARGETS social_navigation_ui_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)

# Export package information
ament_package()

