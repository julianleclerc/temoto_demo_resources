#pragma once

#include <string>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// Function declaration to be called from get_coordinates.cpp
json findCoordinates(
    const std::string& map_path,
    const std::string& items_json_path,
    const std::string& map_yaml_path,
    const std::string& output_dir,
    const std::string& object_description,
    const json& robot_position = json()
);