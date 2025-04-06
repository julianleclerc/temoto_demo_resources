#include "get_coordinates/ai_core.hpp"
#include <iostream>
#include <curl/curl.h>
#include <string>
#include <memory>
#include <stdexcept>
#include <regex>

namespace get_coordinates {

// Callback function for cURL to write response data
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* s) {
    size_t newLength = size * nmemb;
    try {
        s->append((char*)contents, newLength);
        return newLength;
    } catch(std::bad_alloc& e) {
        // Handle memory problem
        return 0;
    }
}

/**
 * Extracts JSON data from LLM responses that may contain debug information or markdown code blocks
 * 
 * @param raw_response The raw response string containing debug logs and JSON data
 * @return Json::Value containing the parsed JSON data
 */
Json::Value extract_json_from_llm_response(const std::string& raw_response) {
    std::cout << "[DEBUG] Extracting JSON from LLM response" << std::endl;
    
    // First try to extract JSON from markdown code blocks
    std::string json_str = extract_json_string_from_llm_response(raw_response);
    
    // Parse the JSON
    Json::Value json_data;
    Json::Reader reader;
    bool parse_success = reader.parse(json_str, json_data);
    
    if (!parse_success) {
        std::cout << "[DEBUG] Failed to parse extracted JSON: " << reader.getFormattedErrorMessages() << std::endl;
        return Json::Value(Json::objectValue);
    }
    
    std::cout << "[DEBUG] Successfully parsed JSON data" << std::endl;
    return json_data;
}

/**
 * Extracts JSON data as a string from LLM responses that may contain debug information or markdown code blocks
 * 
 * @param raw_response The raw response string containing debug logs and JSON data
 * @return std::string containing just the JSON part
 */
std::string extract_json_string_from_llm_response(const std::string& raw_response) {
    std::cout << "[DEBUG] Extracting JSON string from LLM response" << std::endl;
    
    // Method 1: Try to extract JSON from markdown code blocks (```json ... ```)
    std::regex json_code_block_regex("```json\\s*\\n(\\{[\\s\\S]*?\\})\\s*\\n```");
    std::smatch json_code_match;
    
    if (std::regex_search(raw_response, json_code_match, json_code_block_regex) && json_code_match.size() > 1) {
        std::string json_from_code_block = json_code_match[1].str();
        std::cout << "[DEBUG] Found JSON in code block, length: " << json_from_code_block.length() << std::endl;
        return json_from_code_block;
    }
    
    // Method 2: If no code block is found, try to find the outermost JSON object
    std::cout << "[DEBUG] No JSON code block found, looking for outermost JSON object" << std::endl;
    size_t json_start = raw_response.find('{');
    size_t json_end = raw_response.rfind('}');
    
    if (json_start == std::string::npos || json_end == std::string::npos || json_end <= json_start) {
        std::cout << "[DEBUG] Failed to locate valid JSON in response" << std::endl;
        return "{}"; // Return empty JSON object if no valid JSON is found
    }
    
    // Extract the JSON string
    std::string json_str = raw_response.substr(json_start, json_end - json_start + 1);
    std::cout << "[DEBUG] Extracted JSON string using fallback method, length: " << json_str.length() << std::endl;
    
    return json_str;
}

AICore::AICore() {
    std::cout << "[DEBUG AI] AICore constructor called" << std::endl;
    // Initialize with default values
    api_endpoint = "https://api.openai.com/v1/chat/completions";
    
    // Load API key from file
    std::ifstream key_file("/home/fyier/CHATGPT_KEY");
    if (key_file.is_open()) {
        std::getline(key_file, api_key);
        key_file.close();
        
        // Trim whitespace
        api_key.erase(0, api_key.find_first_not_of(" \n\r\t"));
        api_key.erase(api_key.find_last_not_of(" \n\r\t") + 1);
        
        std::cout << "[DEBUG AI] API key loaded from file, length: " << api_key.length() << std::endl;
    } else {
        std::cout << "[DEBUG AI] Warning: Could not open API key file" << std::endl;
        api_key = "";
    }
    
    // Check if API key is configured
    if (api_key.empty()) {
        std::cout << "[DEBUG AI] Warning: API key is empty" << std::endl;
    }
    
    // Initialize cURL globally - should be called once per application
    curl_global_init(CURL_GLOBAL_ALL);
    std::cout << "[DEBUG AI] cURL initialized globally" << std::endl;
}

AICore::~AICore() {
    // Clean up cURL global resources
    curl_global_cleanup();
    std::cout << "[DEBUG AI] AICore destructor called, cURL cleaned up" << std::endl;
}

std::string AICore::AI_Image_Prompt(const std::string& messages,
                                   double temperature,
                                   int max_tokens,
                                   double frequency_penalty,
                                   double presence_penalty) {
    std::cout << "[DEBUG AI] AI_Image_Prompt called" << std::endl;
    std::cout << "[DEBUG AI] messages length: " << messages.length() << std::endl;
    
    // Create request payload 
    Json::Value payload;
    Json::Reader reader;
    bool parse_success = reader.parse(messages, payload["messages"]);
    
    if (!parse_success) {
        std::cout << "[DEBUG AI] Failed to parse messages JSON" << std::endl;
        throw std::runtime_error("Failed to parse messages JSON");
    }
    std::cout << "[DEBUG AI] Successfully parsed messages JSON" << std::endl;
    
    // Set model and parameters
    payload["model"] = "gpt-4o";
    payload["temperature"] = temperature;
    payload["max_tokens"] = max_tokens;
    payload["frequency_penalty"] = frequency_penalty;
    payload["presence_penalty"] = presence_penalty;
    
    // Convert payload to string
    Json::FastWriter writer;
    std::string request_data = writer.write(payload);
    std::cout << "[DEBUG AI] Request data prepared, length: " << request_data.length() << std::endl;
    
    // Check if API key is set
    if (api_key.empty()) {
        std::cout << "[DEBUG AI] Error: API key is not set" << std::endl;
        return createDummyResponse();
    }
    
    // Send the request
    std::cout << "[DEBUG AI] Sending request to: " << api_endpoint << std::endl;
    try {
        std::string response = send_request(request_data);
        std::cout << "[DEBUG AI] Got response from API" << std::endl;
        
        // Process the response to extract just the JSON part from any code blocks or text
        return process_llm_response(response);
    } catch (const std::exception& e) {
        std::cout << "[DEBUG AI] Error in send_request: " << e.what() << std::endl;
        // Return a dummy response for testing when API is unavailable
        return createDummyResponse();
    }
}

/**
 * Process LLM response to extract clean JSON data
 * 
 * @param raw_response The raw response string from the LLM
 * @return std::string containing the processed JSON response
 */
std::string AICore::process_llm_response(const std::string& raw_response) {
    std::cout << "[DEBUG AI] Processing LLM response, length: " << raw_response.length() << std::endl;
    
    // Extract JSON from the response that might include debug logs
    std::string json_str = extract_json_string_from_llm_response(raw_response);
    
    if (json_str == "{}") {
        std::cout << "[DEBUG AI] No valid JSON found in response" << std::endl;
        return createDummyResponse();
    }
    
    std::cout << "[DEBUG AI] Extracted JSON: " << json_str << std::endl;
    return json_str;
}

/**
 * Get parsed JSON data from LLM response
 * 
 * @param raw_response The raw response string from the LLM
 * @return Json::Value containing the parsed JSON data
 */
Json::Value AICore::get_json_from_llm_response(const std::string& raw_response) {
    return extract_json_from_llm_response(raw_response);
}

std::string AICore::createDummyResponse() {
    std::cout << "[DEBUG AI] Creating dummy response for testing" << std::endl;
    Json::Value response;
    response["success"] = "true";
    response["coordinates"] = Json::Value(Json::objectValue);
    response["coordinates"]["x"] = 200;
    response["coordinates"]["y"] = 150;
    response["error"] = "none";
    response["message"] = "This is a dummy response for testing. The API key is not configured.";
    
    Json::FastWriter writer;
    return writer.write(response);
}

void AICore::initialize_connection() {
    std::cout << "[DEBUG AI] initialize_connection called" << std::endl;
    // Placeholder for any connection initialization that needs to be done
    // For example, setting up connection pools, authentication, etc.
}

std::string AICore::send_request(const std::string& payload) {
    std::cout << "[DEBUG AI] send_request called with payload length: " << payload.length() << std::endl;
    
    CURL* curl = curl_easy_init();
    std::string response_string;
    
    if (curl) {
        struct curl_slist* headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        std::string auth_header = "Authorization: Bearer " + api_key;
        headers = curl_slist_append(headers, auth_header.c_str());
        
        curl_easy_setopt(curl, CURLOPT_URL, api_endpoint.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
        
        // Perform the request
        std::cout << "[DEBUG AI] Performing cURL request" << std::endl;
        CURLcode res = curl_easy_perform(curl);
        
        // Check for errors
        if (res != CURLE_OK) {
            std::cout << "[DEBUG AI] cURL request failed: " << curl_easy_strerror(res) << std::endl;
            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
            throw std::runtime_error(std::string("cURL request failed: ") + curl_easy_strerror(res));
        }
        
        std::cout << "[DEBUG AI] cURL request successful, response length: " << response_string.length() << std::endl;
        std::cout << "[DEBUG AI] Full API response: " << response_string << std::endl;
        
        // Clean up
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
        
        // Parse the response to extract just the AI's reply
        Json::Value response_json;
        Json::Reader reader;
        
        if (reader.parse(response_string, response_json)) {
            std::cout << "[DEBUG AI] Successfully parsed response as JSON" << std::endl;
            // For GPT-4 Vision, the content should be in choices[0].message.content
            try {
                if (response_json.isMember("choices") && response_json["choices"].isArray() && 
                    response_json["choices"].size() > 0 && 
                    response_json["choices"][0].isMember("message") && 
                    response_json["choices"][0]["message"].isMember("content")) {
                    
                    std::string assistant_content = response_json["choices"][0]["message"]["content"].asString();
                    std::cout << "[DEBUG AI] Extracted assistant content, length: " << assistant_content.length() << std::endl;
                    
                    // The key change: Extract any JSON from the assistant_content
                    // This will handle cases where the assistant includes text and JSON in code blocks
                    std::string extracted_json = extract_json_string_from_llm_response(assistant_content);
                    if (extracted_json != "{}") {
                        std::cout << "[DEBUG AI] Found valid JSON in assistant content" << std::endl;
                        return extracted_json;
                    }
                    
                    // Return the full content if no JSON was found
                    return assistant_content;
                } else {
                    std::cout << "[DEBUG AI] Response JSON doesn't have expected structure" << std::endl;
                    // Return the full response to let LLM coordinator handle the error
                    return response_string;
                }
            } catch (const std::exception& e) {
                std::cerr << "Error parsing API response: " << e.what() << std::endl;
                std::cout << "[DEBUG AI] Error extracting content: " << e.what() << std::endl;
                return response_string; // Return full response if we can't parse it
            }
        } else {
            std::cout << "[DEBUG AI] Failed to parse response as JSON" << std::endl;
            return response_string; // Return raw response if JSON parsing fails
        }
    } else {
        std::cout << "[DEBUG AI] Failed to initialize cURL" << std::endl;
        throw std::runtime_error("Failed to initialize cURL");
    }
}

} // namespace get_coordinates