cmake_minimum_required(VERSION 3.5)
project(inspect)

add_compile_options(-std=c++20)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(temoto_action_engine REQUIRED)
find_package(temoto_action_engine_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(CURL REQUIRED)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Add both source files to the library
add_library(${PROJECT_NAME} SHARED 
    src/inspect.cpp
    src/ai_core.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} 
    temoto_action_engine
    ${OpenCV_LIBS}
    ${CURL_LIBRARIES}
    fmt::fmt
    ${Boost_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
    temoto_action_engine_ros2
    rclcpp  
    cv_bridge
    sensor_msgs
    nlohmann_json
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

ament_package()