#ifndef AI_CORE_HPP
#define AI_CORE_HPP

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

namespace ai_core {

// Structure to represent an API message
struct Message {
    std::string role;
    std::string content;
};

// Get the OpenAI API key from the environment variable
std::string getApiKey();

// Encode an image to base64 format
std::string encodeImageToBase64(const cv::Mat& image);

// Create a message that includes an image
Message createImageMessage(const std::string& text, const std::string& base64Image);

// Extract valid JSON from text that might contain additional content
std::string extractJsonFromText(const std::string& text);

// Call the OpenAI API with a set of messages
std::string callOpenAIAPI(
    const std::vector<Message>& messages,
    float temperature = 0.3f,
    int max_tokens = 800,
    float frequency_penalty = 0.0f,
    float presence_penalty = 0.0f
);

// Call the OpenAI API with a prompt and an image
std::string AIImagePrompt(
    const std::vector<Message>& messages,
    const cv::Mat& image,
    float temperature = 0.3f,
    int max_tokens = 800,
    float frequency_penalty = 0.0f,
    float presence_penalty = 0.0f
);

}  // namespace ai_core

#endif  // AI_CORE_HPP