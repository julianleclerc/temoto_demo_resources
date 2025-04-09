cmake_minimum_required(VERSION 3.8)
project(trigger_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(temoto_action_engine REQUIRED)
find_package(temoto_action_engine_ros2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(class_loader REQUIRED)

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} temoto_action_engine)

# Link libraries
ament_target_dependencies(${PROJECT_NAME}
  rclcpp  
  std_srvs
  temoto_action_engine
  temoto_action_engine_ros2
  class_loader
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

ament_package()
