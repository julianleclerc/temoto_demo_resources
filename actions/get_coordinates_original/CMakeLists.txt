cmake_minimum_required(VERSION 3.5)
project(get_coordinates)
add_compile_options(-std=c++20)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(temoto_action_engine REQUIRED)
find_package(temoto_action_engine_ros2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(pluginlib REQUIRED)

# Print include dirs for debugging
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "CURL_INCLUDE_DIRS: ${CURL_INCLUDE_DIRS}")

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
  src/get_coordinates_run.cpp
  src/ai_core.cpp
  src/getcoord_costmap_generation.cpp
  src/getcoord_grid_generation.cpp
  src/getcoord_newcoordmap_generation.cpp
  src/getcoord_nonTraversable_generation.cpp
  src/getcoord_objectmap_generation.cpp
  src/getcoord_origincoord_return.cpp
  src/getcoord_pathfind_return.cpp
  src/getcoord_pixelcoord_return.cpp
  src/getcoord_robotmap_generation.cpp
  src/getcoord_scalemap_generation.cpp
  src/llm_coordinator.cpp
)

# Fix the include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${JSONCPP_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
)

# Fix the link libraries
target_link_libraries(${PROJECT_NAME}
  temoto_action_engine
  ${OpenCV_LIBS}
  yaml-cpp
  fmt::fmt
  jsoncpp
  ${CURL_LIBRARIES}
  nlohmann_json::nlohmann_json
)

ament_target_dependencies(${PROJECT_NAME}
  temoto_action_engine_ros2
  ament_index_cpp
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

ament_package()