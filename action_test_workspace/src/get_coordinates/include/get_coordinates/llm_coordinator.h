#ifndef LLM_COORDINATOR_H
#define LLM_COORDINATOR_H

#include <string>
#include <json/json.h>
#include "get_coordinates/ai_core.h"

namespace get_coordinates {

class LLMCoordinator {
public:
    LLMCoordinator();
    ~LLMCoordinator();

    /**
     * Initializes the coordinator with data and instructions
     * 
     * @param data_json JSON containing object classes and items data
     * @param instructions System instructions for the AI
     * @param map_data_str String representation of map data
     */
    void initialize(const Json::Value& data_json, 
                    const std::string& instructions,
                    const std::string& map_data_str);

    /**
     * Search for object coordinates using AI
     * 
     * @param message JSON containing class and description
     * @param object_map JSON or base64 string of the map image
     * @return JSON string with coordinates or error information
     */
    std::string getcoord_search(const Json::Value& message, const Json::Value& object_map);

private:
    Json::Value data;
    std::string INSTRUCTIONS;
    std::string map_data;
    AICore ai_core;

    /**
     * Use LLM to search for coordinates of an object
     * 
     * @param object_class The class of object to find
     * @param object_description Description of the object
     * @param error_log Any previous error logs
     * @param object_map The map image as base64
     * @return JSON string with coordinates or error
     */
    std::string LLM_Search(const std::string& object_class, 
                          const std::string& object_description, 
                          const std::string& error_log, 
                          const Json::Value& object_map);
                          
    // Logging utilities
    void log_info(const std::string& message);
    void log_warn(const std::string& message);
    void log_debug(const std::string& message);
    void log_error(const std::string& message);
};

} // namespace get_coordinates

#endif // LLM_COORDINATOR_H