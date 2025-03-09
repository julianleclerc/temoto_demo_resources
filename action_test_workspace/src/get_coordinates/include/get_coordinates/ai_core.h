#ifndef AI_CORE_H
#define AI_CORE_H

#include <string>
#include <json/json.h>
#include <fstream>

namespace get_coordinates {

// Standalone utility functions for JSON extraction
Json::Value extract_json_from_llm_response(const std::string& raw_response);
std::string extract_json_string_from_llm_response(const std::string& raw_response);

class AICore {
private:
    std::string api_endpoint;
    std::string api_key;
    
    // Helper methods
    std::string send_request(const std::string& payload);
    std::string createDummyResponse();

public:
    AICore();
    ~AICore();
    
    void initialize_connection();
    std::string AI_Image_Prompt(const std::string& messages,
                               double temperature,
                               int max_tokens,
                               double frequency_penalty,
                               double presence_penalty);
                               
    // New methods for handling LLM responses
    std::string process_llm_response(const std::string& raw_response);
    Json::Value get_json_from_llm_response(const std::string& raw_response);
};

} // namespace get_coordinates

#endif // AI_CORE_H