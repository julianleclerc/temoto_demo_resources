cmake_minimum_required(VERSION 3.8)
project(navigate_to_pose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(temoto_action_engine REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(class_loader REQUIRED)

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC include)

# Link libraries
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  class_loader
  rclcpp_action
  nav2_msgs
  temoto_action_engine
)

class_loader_hide_library_symbols(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

ament_package()
