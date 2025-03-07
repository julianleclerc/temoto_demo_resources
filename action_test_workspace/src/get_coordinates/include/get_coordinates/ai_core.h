#ifndef AI_CORE_H
#define AI_CORE_H

#include <string>
#include <json/json.h>
#include <fstream>

namespace get_coordinates {

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
};

} // namespace get_coordinates

#endif // AI_CORE_H