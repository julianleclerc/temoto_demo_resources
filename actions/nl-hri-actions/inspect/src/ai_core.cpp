#include "inspect/ai_core.hpp"

#include <cstdlib>
#include <iostream>
#include <vector>
#include <curl/curl.h>
#include <opencv2/opencv.hpp>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/transform_width.hpp>

namespace ai_core {

// Callback function for CURL
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* s) {
    size_t newLength = size * nmemb;
    try {
        s->append(static_cast<char*>(contents), newLength);
        return newLength;
    } catch(std::bad_alloc& e) {
        // Handle memory problem    
        return 0;
    }
}

std::string getApiKey() {
    std::cout << "AI Core: Getting OpenAI API key from environment..." << std::endl;
    
    const char* apiKey = std::getenv("OPENAI_API_KEY");
    if (!apiKey) {
        std::cerr << "AI Core: OPENAI_API_KEY environment variable not set!" << std::endl;
        throw std::runtime_error("OPENAI_API_KEY environment variable not set");
    }
    
    std::cout << "AI Core: API key retrieved successfully" << std::endl;
    return std::string(apiKey);
}

std::string encodeImageToBase64(const cv::Mat& image) {
    std::cout << "AI Core: Starting image encoding to base64..." << std::endl;
    
    try {
        // Set JPEG compression parameters for smaller file size
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(65); 
        
        std::vector<uchar> buf;
        std::cout << "AI Core: Compressing image to JPEG format..." << std::endl;
        bool success = cv::imencode(".jpg", image, buf, compression_params);
        
        if (!success) {
            std::cerr << "AI Core: Failed to encode image to JPEG format" << std::endl;
            return "";
        }
        
        std::cout << "AI Core: JPEG compression successful, size: " << buf.size() << " bytes" << std::endl;
        
        using namespace boost::archive::iterators;
        using base64_text = base64_from_binary<transform_width<const char *, 6, 8>>;
        
        std::cout << "AI Core: Converting to base64..." << std::endl;
        std::string base64_image(base64_text((char *)buf.data()), 
                               base64_text((char *)buf.data() + buf.size()));
        
        // Add padding if needed
        size_t padding = (3 - buf.size() % 3) % 3;
        for (size_t i = 0; i < padding; i++) {
            base64_image.push_back('=');
        }
        
        std::cout << "AI Core: Base64 encoding complete, output size: " << base64_image.size() << " bytes" << std::endl;
        
        return base64_image;
    } catch (const std::exception& e) {
        std::cerr << "AI Core: Exception in base64 encoding: " << e.what() << std::endl;
        return "";
    }
}

Message createImageMessage(const std::string& text, const std::string& base64Image) {
    std::cout << "AI Core: Creating image message with text: " << text << std::endl;
    std::cout << "AI Core: Base64 image size: " << base64Image.size() << " bytes" << std::endl;
    
    // Create a JSON array for content that includes both text and image
    nlohmann::json content = nlohmann::json::array();
    
    // Add the text part
    content.push_back({
        {"type", "text"},
        {"text", text}
    });
    
    // Add the image part with high detail
    content.push_back({
        {"type", "image_url"},
        {"image_url", {
            {"url", "data:image/jpeg;base64," + base64Image},
            {"detail", "high"}
        }}
    });
    
    std::cout << "AI Core: Created JSON content array for message" << std::endl;
    
    // Return a Message with JSON content
    return Message("user", content);
}

std::string callOpenAIAPI(
    const std::vector<Message>& messages,
    float temperature,
    int max_tokens,
    float frequency_penalty,
    float presence_penalty) {
    
    std::cout << "AI Core: Calling OpenAI API..." << std::endl;
    std::cout << "AI Core: Number of messages: " << messages.size() << std::endl;
    std::cout << "AI Core: Temperature: " << temperature << ", Max tokens: " << max_tokens << std::endl;
    
    try {
        std::cout << "AI Core: Getting API key..." << std::endl;
        std::string apiKey;
        try {
            apiKey = getApiKey();
        } catch (const std::exception& e) {
            std::cerr << "AI Core: Failed to get API key: " << e.what() << std::endl;
            nlohmann::json error_json = {
                {"success", false},
                {"message", std::string("API key error: ") + e.what()}
            };
            return error_json.dump();
        }
        
        // Initialize CURL
        std::cout << "AI Core: Initializing CURL..." << std::endl;
        CURL* curl = curl_easy_init();
        std::string response_string;
        
        if (curl) {
            std::cout << "AI Core: CURL initialized successfully" << std::endl;
            
            // Set URL
            std::cout << "AI Core: Setting CURL URL to OpenAI API endpoint..." << std::endl;
            curl_easy_setopt(curl, CURLOPT_URL, "https://api.openai.com/v1/chat/completions");
            
            // Set headers
            std::cout << "AI Core: Setting CURL headers..." << std::endl;
            struct curl_slist* headers = NULL;
            headers = curl_slist_append(headers, "Content-Type: application/json");
            std::string auth_header = "Authorization: Bearer " + apiKey;
            headers = curl_slist_append(headers, auth_header.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
            
            // Create request JSON
            std::cout << "AI Core: Creating request JSON..." << std::endl;
            nlohmann::json request_json;
            request_json["model"] = "gpt-4o";
            request_json["temperature"] = temperature;
            request_json["max_tokens"] = max_tokens;
            request_json["top_p"] = 0.5;
            request_json["frequency_penalty"] = frequency_penalty;
            request_json["presence_penalty"] = presence_penalty;
            
            // Add messages
            nlohmann::json message_array = nlohmann::json::array();
            for (const auto& message : messages) {
                if (message.is_json_content) {
                    // Use structured JSON content
                    message_array.push_back({
                        {"role", message.role},
                        {"content", message.content_json}
                    });
                    std::cout << "AI Core: Added message with role: " << message.role 
                            << ", using structured JSON content" << std::endl;
                } else {
                    // Use plain text content
                    message_array.push_back({
                        {"role", message.role},
                        {"content", message.content}
                    });
                    std::cout << "AI Core: Added message with role: " << message.role 
                            << ", content as text, length: " << message.content.size() << " bytes" << std::endl;
                }
            }
            request_json["messages"] = message_array;
            
            // Set request data
            std::string request_data = request_json.dump();
            std::cout << "AI Core: Request JSON created, size: " << request_data.size() << " bytes" << std::endl;
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_data.c_str());
            
            // Set response callback
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
            
            // Perform request
            std::cout << "AI Core: Performing CURL request to OpenAI API..." << std::endl;
            CURLcode res = curl_easy_perform(curl);
            
            // Check for errors
            if (res != CURLE_OK) {
                std::cerr << "AI Core: CURL error: " << curl_easy_strerror(res) << std::endl;
                nlohmann::json error_json = {
                    {"success", false},
                    {"message", std::string("CURL error: ") + curl_easy_strerror(res)}
                };
                
                // Clean up
                curl_slist_free_all(headers);
                curl_easy_cleanup(curl);
                
                return error_json.dump();
            }
            
            std::cout << "AI Core: CURL request completed successfully" << std::endl;
            std::cout << "AI Core: Response size: " << response_string.size() << " bytes" << std::endl;
            
            // Clean up
            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
            
            // Parse response
            try {
                std::cout << "AI Core: Parsing JSON response..." << std::endl;
                nlohmann::json response_json = nlohmann::json::parse(response_string);
                
                if (response_json.contains("choices") && !response_json["choices"].empty() &&
                    response_json["choices"][0].contains("message") &&
                    response_json["choices"][0]["message"].contains("content")) {
                    
                    std::string content = response_json["choices"][0]["message"]["content"].get<std::string>();
                    std::cout << "AI Core: Successfully extracted content from response (length: " 
                              << content.size() << " bytes)" << std::endl;
                    return content;
                } else {
                    std::cerr << "AI Core: Invalid response format from API" << std::endl;
                    if (response_string.size() < 1000) {
                        std::cerr << "AI Core: Response: " << response_string << std::endl;
                    } else {
                        std::cerr << "AI Core: Response too large to print" << std::endl;
                    }
                    
                    nlohmann::json error_json = {
                        {"success", false},
                        {"message", "Invalid response format from API"}
                    };
                    return error_json.dump();
                }
            } catch (const std::exception& e) {
                std::cerr << "AI Core: Error parsing response: " << e.what() << std::endl;
                std::cerr << "AI Core: Response preview: " 
                          << (response_string.size() > 100 ? response_string.substr(0, 100) + "..." : response_string) 
                          << std::endl;
                
                nlohmann::json error_json = {
                    {"success", false},
                    {"message", std::string("Error parsing response: ") + e.what()}
                };
                return error_json.dump();
            }
        } else {
            std::cerr << "AI Core: Failed to initialize CURL" << std::endl;
            nlohmann::json error_json = {
                {"success", false},
                {"message", "Failed to initialize CURL"}
            };
            return error_json.dump();
        }
    } catch (const std::exception& e) {
        std::cerr << "AI Core: Critical error in API call: " << e.what() << std::endl;
        nlohmann::json error_json = {
            {"success", false},
            {"message", std::string("Error sending data to LLM: ") + e.what()}
        };
        return error_json.dump();
    }
}

std::string AIImagePrompt(
    const std::vector<Message>& messages,
    const cv::Mat& image,
    float temperature,
    int max_tokens,
    float frequency_penalty,
    float presence_penalty) {
    
    std::cout << "AI Core: Starting Image Prompt processing..." << std::endl;
    std::cout << "AI Core: Original image dimensions: " << image.cols << "x" << image.rows << std::endl;
    
    try {
        // Resize the image to reduce the token count
        cv::Mat resized_image;
        int max_dimension = 320; // Limit maximum dimension to 400 pixels
        
        double scale = 1.0;
        if (image.cols > image.rows) {
            scale = static_cast<double>(max_dimension) / image.cols;
        } else {
            scale = static_cast<double>(max_dimension) / image.rows;
        }
        
        if (scale < 1.0) {
            int new_width = static_cast<int>(image.cols * scale);
            int new_height = static_cast<int>(image.rows * scale);
            std::cout << "AI Core: Resizing image to " << new_width << "x" << new_height << " to reduce token count" << std::endl;
            cv::resize(image, resized_image, cv::Size(new_width, new_height), 0, 0, cv::INTER_AREA);
        } else {
            std::cout << "AI Core: Image already small enough, no resizing needed" << std::endl;
            resized_image = image.clone();
        }
        
        // Convert image to base64
        std::cout << "AI Core: Converting image to base64..." << std::endl;
        std::string base64Image = encodeImageToBase64(resized_image);
        
        std::cout << "AI Core: Base64 encoding successful, compressed size: " << base64Image.size() << " bytes" << std::endl;
        
        if (base64Image.empty()) {
            std::cerr << "AI Core: Failed to encode image to base64" << std::endl;
            nlohmann::json error_json = {
                {"success", false},
                {"message", "Failed to encode image to base64"}
            };
            return error_json.dump();
        }
        
        // Create modified messages for the API call
        std::cout << "AI Core: Creating message structure for OpenAI API..." << std::endl;
        std::vector<Message> api_messages;
        
        // Copy system messages as is
        for (const auto& msg : messages) {
            if (msg.role == "system") {
                api_messages.push_back(msg);
                std::cout << "AI Core: Added system message" << std::endl;
            }
        }
        
        // Create a user message with the image
        std::string promptText = "Inspect this image.";
        if (!messages.empty()) {
            // Get prompt from the last user message if available
            for (auto it = messages.rbegin(); it != messages.rend(); ++it) {
                if (it->role == "user") {
                    promptText = it->content;
                    break;
                }
            }
        }
        
        // Construct a multipart message with text and image
        api_messages.push_back(createImageMessage(promptText, base64Image));
        std::cout << "AI Core: Created user message with image content" << std::endl;
        
        // Use our helper function to make the API call
        std::cout << "AI Core: Calling OpenAI API with " << api_messages.size() << " messages" << std::endl;
        return callOpenAIAPI(
            api_messages,
            temperature,
            max_tokens,
            frequency_penalty,
            presence_penalty
        );
    } catch (const std::exception& e) {
        std::cerr << "AI Core: Error in image prompt processing: " << e.what() << std::endl;
        nlohmann::json error_json = {
            {"success", false},
            {"message", std::string("Error sending image to LLM: ") + e.what()}
        };
        return error_json.dump();
    }
}

}  // namespace ai_core