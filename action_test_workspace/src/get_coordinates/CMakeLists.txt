cmake_minimum_required(VERSION 3.5)
project(get_coordinates)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Find nlohmann_json
find_package(nlohmann_json REQUIRED)

# Find YAML-cpp
find_package(yaml-cpp REQUIRED)

# Find libcurl (needed for AI HTTP requests)
find_package(CURL REQUIRED)

# Find jsoncpp (needed for AI JSON handling)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

# Sources
set(SOURCES
  src/get_coordinates_run.cpp
  src/getcoord_scalemap_generation.cpp
  src/getcoord_costmap_generation.cpp
  src/getcoord_nonTraversable_generation.cpp
  src/getcoord_grid_generation.cpp
  src/getcoord_objectmap_generation.cpp
  src/getcoord_pixelcoord_return.cpp
  src/getcoord_newcoordmap_generation.cpp
  src/getcoord_origincoord_return.cpp
  src/getcoord_pathfind_return.cpp
  src/getcoord_robotmap_generation.cpp
  src/ai_core.cpp
  src/llm_coordinator.cpp
)

# Executable
add_executable(get_coordinates ${SOURCES})

# Link libraries
target_link_libraries(get_coordinates
  ${OpenCV_LIBS}
  nlohmann_json::nlohmann_json
  yaml-cpp
  ${CURL_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)

# Installation
install(TARGETS get_coordinates
  RUNTIME DESTINATION bin
)