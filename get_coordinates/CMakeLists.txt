cmake_minimum_required(VERSION 3.5)
project(get_coordinates)
add_compile_options(-std=c++17)

# Find dependencies
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

# Define the common source files (excluding main files)
set(COMMON_SOURCES
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

# Only build the test executable
add_executable(get_coordinates_run_test 
  src/get_coordinates_run.cpp
  ${COMMON_SOURCES}
)

# Get JsonCpp include directories
execute_process(
  COMMAND pkg-config --cflags jsoncpp
  OUTPUT_VARIABLE JSONCPP_CFLAGS
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Include directories
target_include_directories(get_coordinates_run_test PUBLIC 
  include
  data
  ${JSONCPP_CFLAGS}
)

target_link_libraries(get_coordinates_run_test
  ${OpenCV_LIBRARIES}
  yaml-cpp
  fmt::fmt
  jsoncpp
  ${CURL_LIBRARIES}
  nlohmann_json::nlohmann_json
)

# Installation
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY data/ DESTINATION share/${PROJECT_NAME}/data)