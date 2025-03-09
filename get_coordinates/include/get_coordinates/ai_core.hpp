#pragma once

#include <string>
#include <fstream>
#include <jsoncpp/json/json.h> // Changed from json/json.h to jsoncpp/json/json.h

namespace get_coordinates {

// Function declarations
Json::Value extract_json_from_llm_response(const std::string& raw_response);
std::string extract_json_string_from_llm_response(const std::string& raw_response);

class AICore {
public:
    AICore();
    ~AICore();
    
    // Main API call method for the LLM (with image)
    std::string AI_Image_Prompt(const std::string& messages,
                               double temperature = 1.0,
                               int max_tokens = 300,
                               double frequency_penalty = 0.0,
                               double presence_penalty = 0.0);
    
    // Process the LLM response to extract clean JSON data
    std::string process_llm_response(const std::string& raw_response);
    
    // Get parsed JSON data from LLM response
    Json::Value get_json_from_llm_response(const std::string& raw_response);
    
    // Initialize connection to the API
    void initialize_connection();

private:
    // API endpoint and key
    std::string api_endpoint;
    std::string api_key;
    
    // Send the HTTP request to the API
    std::string send_request(const std::string& payload);
    
    // Create a dummy response for testing
    std::string createDummyResponse();
};

} // namespace get_coordinates