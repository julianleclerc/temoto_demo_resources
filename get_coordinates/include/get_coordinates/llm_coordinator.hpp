#pragma once

#include <string>
#include <jsoncpp/json/json.h> // Changed from json/json.h to jsoncpp/json/json.h
#include "get_coordinates/ai_core.hpp"

namespace get_coordinates {

class LLMCoordinator {
public:
    LLMCoordinator();
    ~LLMCoordinator();
    
    /**
     * Initialize the LLMCoordinator with data
     * 
     * @param data_json The JSON data with items information
     * @param instructions The system instructions for the LLM
     * @param map_data_str The map data as a string
     */
    void initialize(const Json::Value& data_json, 
                   const std::string& instructions,
                   const std::string& map_data_str);
    
    /**
     * Search for coordinates based on object class and description
     * 
     * @param message The JSON message containing object class and description
     * @param object_map The base64-encoded image of the object map
     * @return std::string The JSON response with coordinates
     */
    std::string getcoord_search(const Json::Value& message, const Json::Value& object_map);

private:
    // AI core for API calls
    AICore ai_core;
    
    // Data storage
    Json::Value data;
    std::string map_data;
    std::string INSTRUCTIONS;
    
    /**
     * Search for coordinates using the LLM
     * 
     * @param object_class The object class to search for
     * @param object_description The object description
     * @param error_log Any error logs from previous attempts
     * @param object_map The base64-encoded image of the object map
     * @return std::string The JSON response with coordinates
     */
    std::string LLM_Search(const std::string& object_class, 
                          const std::string& object_description, 
                          const std::string& error_log, 
                          const Json::Value& object_map);
    
    // Logging methods
    void log_info(const std::string& message);
    void log_warn(const std::string& message);
    void log_debug(const std::string& message);
    void log_error(const std::string& message);
};

} // namespace get_coordinates