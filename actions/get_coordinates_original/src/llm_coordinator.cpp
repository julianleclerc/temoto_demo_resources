#include "get_coordinates/llm_coordinator.hpp"
#include <iostream>

namespace get_coordinates {

LLMCoordinator::LLMCoordinator() {
    // Default constructor
    std::cout << "[DEBUG LLM] LLMCoordinator constructor called" << std::endl;
}

LLMCoordinator::~LLMCoordinator() {
    // Default destructor
    std::cout << "[DEBUG LLM] LLMCoordinator destructor called" << std::endl;
}

void LLMCoordinator::initialize(const Json::Value& data_json, 
                               const std::string& instructions,
                               const std::string& map_data_str) {
    std::cout << "[DEBUG LLM] initialize called" << std::endl;
    this->data = data_json;
    this->INSTRUCTIONS = instructions;
    this->map_data = map_data_str;
    std::cout << "[DEBUG LLM] initialize completed" << std::endl;
}

std::string LLMCoordinator::getcoord_search(const Json::Value& message, const Json::Value& object_map) {
    std::cout << "[DEBUG LLM] getcoord_search called" << std::endl;
    
    // Get message attributes - just use the description field
    std::string object_description = message.get("description", " ").asString();
    std::string error_log = "";
    
    std::cout << "[DEBUG LLM] Got object_description: " << object_description << std::endl;
    
    // Call LLM_Search and find coordinates
    std::cout << "[DEBUG LLM] Calling LLM_Search to find coordinates" << std::endl;
    log_info("Requesting coord from AI with description: " + object_description);
    std::string assistant_reply = LLM_Search(object_description, error_log, object_map);
    
    std::cout << "[DEBUG LLM] Got LLM_Search response, length: " << assistant_reply.length() << std::endl;
    if (assistant_reply.empty()) {
        std::cout << "[DEBUG LLM] Warning: Empty assistant_reply" << std::endl;
        Json::Value empty_response;
        empty_response["success"] = "false";
        empty_response["error"] = "emptyResponse";
        empty_response["message"] = "The AI returned an empty response";
        
        std::string error_reply = Json::FastWriter().write(empty_response);
        log_warn("Empty response from AI, returning error: " + error_reply);
        return error_reply;
    }
    
    Json::Value assistant_reply_json;
    Json::Reader reader;
    std::cout << "[DEBUG LLM] Parsing assistant_reply: " << assistant_reply << std::endl;
    bool parse_success = reader.parse(assistant_reply, assistant_reply_json);
    
    if (!parse_success) {
        std::cout << "[DEBUG LLM] Failed to parse assistant_reply as JSON" << std::endl;
        // Create a valid JSON response with the raw text
        Json::Value error_response;
        error_response["success"] = "false";
        error_response["error"] = "parseError";
        error_response["message"] = "Failed to parse AI response";
        error_response["raw_response"] = assistant_reply;
        
        std::string error_reply = Json::FastWriter().write(error_response);
        log_warn("Parse error, returning: " + error_reply);
        return error_reply;
    }
    
    log_debug("Sending response: " + assistant_reply);
    std::cout << "[DEBUG LLM] Returning LLM_Search response" << std::endl;
    return assistant_reply;
}

std::string LLMCoordinator::LLM_Search(const std::string& object_description, 
                                      const std::string& error_log, 
                                      const Json::Value& object_map) {
    std::cout << "[DEBUG LLM] LLM_Search called for description: " << object_description << std::endl;
        
    // Initialize messages with system instructions
    Json::Value messages(Json::arrayValue);

    // System instruction
    Json::Value systemMsg;
    systemMsg["role"] = "system";

    Json::Value systemContent(Json::arrayValue);
    Json::Value systemTextContent;
    systemTextContent["type"] = "text";
    systemTextContent["text"] = INSTRUCTIONS;
    systemContent.append(systemTextContent);

    systemMsg["content"] = systemContent;
    messages.append(systemMsg);

    // Add the object list
    Json::Value objectListMsg;
    objectListMsg["role"] = "user";

    Json::Value objectListContent(Json::arrayValue);
    Json::Value objectListTextContent;
    objectListTextContent["type"] = "text";
    objectListTextContent["text"] = "The list of objects registered are: " + map_data;
    objectListContent.append(objectListTextContent);

    objectListMsg["content"] = objectListContent;
    messages.append(objectListMsg);

    // Add the map
    Json::Value mapMsg;
    mapMsg["role"] = "user";

    Json::Value mapContent(Json::arrayValue);
    Json::Value mapTextContent;
    mapTextContent["type"] = "text";
    mapTextContent["text"] = "The map is: ";
    mapContent.append(mapTextContent);

    Json::Value mapImageContent;
    mapImageContent["type"] = "image_url";

    Json::Value imageUrl;
    // Assuming object_map is already a base64 string
    std::string base64Map = object_map.asString();
    std::cout << "[DEBUG LLM] base64Map length: " << base64Map.length() << std::endl;
    imageUrl["url"] = "data:image/jpeg;base64," + base64Map;

    mapImageContent["image_url"] = imageUrl;
    mapContent.append(mapImageContent);

    mapMsg["content"] = mapContent;
    messages.append(mapMsg);

    // Add the object description
    Json::Value objectMsg;
    objectMsg["role"] = "user";

    Json::Value objectContent(Json::arrayValue);
    Json::Value objectTextContent;
    objectTextContent["type"] = "text";
    objectTextContent["text"] = "Return the coordinates for object with description: " + object_description;
    objectContent.append(objectTextContent);

    objectMsg["content"] = objectContent;
    messages.append(objectMsg);

    // Add error log if it exists
    if (!error_log.empty()) {
        std::cout << "[DEBUG LLM] Adding error log" << std::endl;
        std::string error_info = "\n An error has previously come up, below is the thread of messages between you and the user.\n"
            " Please use this information and try to determine the correct object and return its coordinates.\n"
            " If the object is still not clear, continue until success or until user asks to skip.\n";

        Json::Value errorInfoMsg;
        errorInfoMsg["role"] = "system";

        Json::Value errorInfoContent(Json::arrayValue);
        Json::Value errorInfoTextContent;
        errorInfoTextContent["type"] = "text";
        errorInfoTextContent["text"] = error_info;
        errorInfoContent.append(errorInfoTextContent);

        errorInfoMsg["content"] = errorInfoContent;
        messages.append(errorInfoMsg);

        Json::Value errorLogMsg;
        errorLogMsg["role"] = "assistant";

        Json::Value errorLogContent(Json::arrayValue);
        Json::Value errorLogTextContent;
        errorLogTextContent["type"] = "text";
        errorLogTextContent["text"] = error_log;
        errorLogContent.append(errorLogTextContent);

        errorLogMsg["content"] = errorLogContent;
        messages.append(errorLogMsg);
    }

    // Get response from AI
    try {
        // Call to AI service
        std::cout << "[DEBUG LLM] Preparing to call AI_Image_Prompt" << std::endl;
        std::string messages_json = Json::FastWriter().write(messages);
        std::cout << "[DEBUG LLM] Messages JSON prepared, length: " << messages_json.length() << std::endl;
        
        std::string assistant_reply = ai_core.AI_Image_Prompt(
            messages_json,
            1.0,    // TEMPERATURE
            300,    // MAX_TOKENS
            0.0,    // FREQUENCY_PENALTY
            0.0     // PRESENCE_PENALTY
        );

        std::cout << "[DEBUG LLM] Received assistant_reply from AI, length: " << assistant_reply.length() << std::endl;
        log_info("assistant_reply: " + assistant_reply);
        
        // If the response is empty, provide a fallback
        if (assistant_reply.empty()) {
            std::cout << "[DEBUG LLM] assistant_reply is empty, returning fallback" << std::endl;
            Json::Value fallbackResponse;
            fallbackResponse["success"] = "false";
            fallbackResponse["error"] = "emptyResponse";
            fallbackResponse["message"] = "AI returned an empty response";
            
            return Json::FastWriter().write(fallbackResponse);
        }
        
        // Try to validate if the response is a valid JSON
        Json::Value validation;
        Json::Reader reader;
        if (!reader.parse(assistant_reply, validation)) {
            std::cout << "[DEBUG LLM] Response is not valid JSON, wrapping it" << std::endl;
            std::cout << "[DEBUG LLM] Raw response: " << assistant_reply << std::endl;
            Json::Value wrappedResponse;
            wrappedResponse["success"] = "false";
            wrappedResponse["error"] = "invalidJSON";
            wrappedResponse["raw_response"] = assistant_reply;
            wrappedResponse["message"] = "AI did not return a valid JSON response";
            
            return Json::FastWriter().write(wrappedResponse);
        }
        
        // Check if this is an error response from the API
        if (validation.isMember("error") && validation["error"].asString() != "none") {
            std::cout << "[DEBUG LLM] API returned an error" << std::endl;
            Json::Value errorResponse;
            errorResponse["success"] = "false";
            errorResponse["error"] = "apiError";
            errorResponse["message"] = "API returned an error";
            if (validation["error"].isObject() && validation["error"].isMember("message")) {
                errorResponse["error_details"] = validation["error"]["message"];
            } else {
                errorResponse["error_details"] = validation["error"].asString();
            }
            
            return Json::FastWriter().write(errorResponse);
        }
        
        // Provide default coordinates if none are in the response
        if (!validation.isMember("coordinates")) {
            std::cout << "[DEBUG LLM] Response missing coordinates, adding defaults" << std::endl;
            validation["coordinates"] = Json::Value(Json::objectValue);
            validation["coordinates"]["x"] = 0;
            validation["coordinates"]["y"] = 0;
            validation["success"] = "false";
            validation["error"] = "noCoordinates";
            validation["message"] = "AI response did not include coordinates";
            
            return Json::FastWriter().write(validation);
        }
        
        return assistant_reply;
    } 
    catch (const std::exception& e) {
        std::cout << "[DEBUG LLM] Exception in AI_Image_Prompt: " << e.what() << std::endl;
        log_info("Error sending data to LLM: " + std::string(e.what()));

        Json::Value errorResponse;
        errorResponse["success"] = "false";
        errorResponse["error"] = "skip";
        errorResponse["message"] = "Error sending data to LLM: " + std::string(e.what());

        std::string error_reply = Json::FastWriter().write(errorResponse);
        log_info("assistant_reply: " + error_reply);

        return error_reply;
    }
}

// Logging methods
void LLMCoordinator::log_info(const std::string& message) {
    std::cout << "[INFO] " << message << std::endl;
}

void LLMCoordinator::log_warn(const std::string& message) {
    std::cout << "[WARN] " << message << std::endl;
}

void LLMCoordinator::log_debug(const std::string& message) {
    std::cout << "[DEBUG] " << message << std::endl;
}

void LLMCoordinator::log_error(const std::string& message) {
    std::cerr << "[ERROR] " << message << std::endl;
}

} // namespace get_coordinates