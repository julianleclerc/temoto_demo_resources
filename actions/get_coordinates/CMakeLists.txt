cmake_minimum_required(VERSION 3.5)
project(get_coordinates)
add_compile_options(-std=c++17)

# Check if this is a ROS environment
set(IS_ROS_ENV FALSE)
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  set(IS_ROS_ENV TRUE)
endif()

# Find dependencies
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

# Conditionally find ROS packages
if(IS_ROS_ENV)
  find_package(class_loader REQUIRED)
endif()

# Add all source files to the library target
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

target_include_directories(${PROJECT_NAME} PUBLIC include data)

# Conditionally link against ROS libraries
if(IS_ROS_ENV)
  target_link_libraries(${PROJECT_NAME} 
    temoto_action_engine
    ${OpenCV_LIBRARIES}
    yaml-cpp
    fmt::fmt
    jsoncpp
    ${CURL_LIBRARIES}
    nlohmann_json::nlohmann_json
  )
  # Use ROS-specific commands
  class_loader_hide_library_symbols(${PROJECT_NAME})
else()
  target_link_libraries(${PROJECT_NAME} 
    ${OpenCV_LIBRARIES}
    yaml-cpp
    fmt::fmt
    jsoncpp
    ${CURL_LIBRARIES}
    nlohmann_json::nlohmann_json
  )
endif()

# Add executable target that uses the same source files
add_executable(run_get_coordinates
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

target_include_directories(run_get_coordinates PUBLIC include data)
target_link_libraries(run_get_coordinates
  ${OpenCV_LIBRARIES}
  yaml-cpp
  fmt::fmt
  jsoncpp
  ${CURL_LIBRARIES}
  nlohmann_json::nlohmann_json
)

# Installation
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(TARGETS run_get_coordinates DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY data/ DESTINATION share/${PROJECT_NAME}/data)

# Conditionally call ament_package()
if(IS_ROS_ENV)
  ament_package()
endif()