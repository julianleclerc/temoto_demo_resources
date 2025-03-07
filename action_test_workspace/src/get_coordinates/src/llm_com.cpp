#include "get_coordinates/llm_com.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstdlib>
#include <curl/curl.h>
#include <filesystem>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using json = nlohmann::json;

// Callback function for cURL to write received data to a string
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}

LlmCom::LlmCom() {
    // Initialize cURL globally once
    curl_global_init(CURL_GLOBAL_ALL);
}

LlmCom::~LlmCom() {
    // Clean up global cURL resources
    curl_global_cleanup();
}

std::string LlmCom::getApiKey() {
    // First try to get from environment variable
    char* env_key = std::getenv("OPENAI_API_KEY");
    if (env_key != nullptr) {
        return std::string(env_key);
    }
    
    // If not in environment, try to read from a file in the home directory
    std::string home_dir = std::getenv("HOME") ? std::getenv("HOME") : ".";
    std::string key_file = home_dir + "/.openai_api_key";
    
    if (fs::exists(key_file)) {
        std::ifstream file(key_file);
        if (file.is_open()) {
            std::string api_key;
            std::getline(file, api_key);
            file.close();
            return api_key;
        }
    }
    
    throw std::runtime_error("OpenAI API key not found in environment or key file");
}

std::string LlmCom::makeOpenAIRequest(const json& request_data) {
    CURL* curl = curl_easy_init();
    std::string response_string;
    
    if (curl) {
        // Set up headers
        struct curl_slist* headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        
        std::string auth_header = "Authorization: Bearer " + getApiKey();
        headers = curl_slist_append(headers, auth_header.c_str());
        
        // Convert request_data to string
        std::string request_string = request_data.dump();
        
        // Set up the request
        curl_easy_setopt(curl, CURLOPT_URL, "https://api.openai.com/v1/chat/completions");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_string.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
        
        // Perform the request
        CURLcode res = curl_easy_perform(curl);
        
        // Clean up
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
        
        if (res != CURLE_OK) {
            throw std::runtime_error(std::string("cURL request failed: ") + curl_easy_strerror(res));
        }
    } else {
        throw std::runtime_error("Failed to initialize cURL");
    }
    
    return response_string;
}

std::string LlmCom::sendImagePrompt(
    const json& messages,
    float temperature,
    int max_tokens,
    float frequency_penalty,
    float presence_penalty
) {
    try {
        // Build the request JSON
        json request_data = {
            {"model", "gpt-4o"},
            {"messages", messages},
            {"temperature", temperature},
            {"max_tokens", max_tokens},
            {"top_p", 1},
            {"frequency_penalty", frequency_penalty},
            {"presence_penalty", presence_penalty}
        };
        
        // Make the API request
        std::string response_str = makeOpenAIRequest(request_data);
        
        // Parse the response
        json response = json::parse(response_str);
        
        // Extract the content from the response
        if (response.contains("choices") && response["choices"].is_array() && 
            !response["choices"].empty() && response["choices"][0].contains("message") && 
            response["choices"][0]["message"].contains("content")) {
            
            return response["choices"][0]["message"]["content"];
        }
        
        // If we can't extract the content properly, return the raw response
        return response_str;
        
    } catch (const std::exception& e) {
        json error_response = {
            {"success", "false"},
            {"message", std::string("Error sending data to LLM: ") + e.what()}
        };
        
        return error_response.dump();
    }
}

json LlmCom::createSystemMessage(const std::string& content) {
    return {
        {"role", "system"},
        {"content", {
            {
                {"type", "text"},
                {"text", content}
            }
        }}
    };
}

json LlmCom::createUserTextMessage(const std::string& content) {
    return {
        {"role", "user"},
        {"content", {
            {
                {"type", "text"},
                {"text", content}
            }
        }}
    };
}

json LlmCom::createUserImageMessage(
    const std::string& text,
    const std::string& base64_image,
    const std::string& detail
) {
    return {
        {"role", "user"},
        {"content", {
            {
                {"type", "text"},
                {"text", text}
            },
            {
                {"type", "image_url"},
                {"image_url", {
                    {"url", "data:image/jpeg;base64," + base64_image},
                    {"detail", detail}
                }}
            }
        }}
    };
}

json LlmCom::createAssistantMessage(const std::string& content) {
    return {
        {"role", "assistant"},
        {"content", {
            {
                {"type", "text"},
                {"text", content}
            }
        }}
    };
}