#ifndef INSPECT_AI_CORE_HPP
#define INSPECT_AI_CORE_HPP

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

namespace ai_core {

/**
 * @struct Message
 * @brief Structure to define messages for AI communication
 */
struct Message {
    std::string role;         // Role of the message (e.g., "system", "user", "assistant")
    std::string content;      // Content of the message as text
    nlohmann::json content_json; // Content of the message as JSON (for multimodal messages)
    bool is_json_content;     // Flag to indicate if content is JSON

    /**
     * @brief Constructor for text-based message
     * @param r Role of the message
     * @param c Content of the message as text
     */
    Message(std::string r, std::string c) 
        : role(r), content(c), is_json_content(false) {}
    
    /**
     * @brief Constructor for JSON-based message (for multimodal content)
     * @param r Role of the message
     * @param c Content of the message as JSON
     */
    Message(std::string r, nlohmann::json c) 
        : role(r), content_json(c), is_json_content(true) {}
};

/**
 * @brief Encodes an OpenCV image to base64 format
 * @param image The image to encode
 * @return Base64-encoded string
 */
std::string encodeImageToBase64(const cv::Mat& image);

/**
 * @brief Creates a message with text and image content
 * @param text Text content of the message
 * @param base64Image Base64-encoded image data
 * @return Message structure with combined content
 */
Message createImageMessage(const std::string& text, const std::string& base64Image);

/**
 * @brief Calls the OpenAI API with a list of messages
 * @param messages Vector of Message structures
 * @param temperature Controls randomness (0.0-1.0)
 * @param max_tokens Maximum tokens to generate
 * @param frequency_penalty Penalizes token frequency (0.0-2.0)
 * @param presence_penalty Penalizes token presence (0.0-2.0)
 * @return API response as a string
 */
std::string callOpenAIAPI(
    const std::vector<Message>& messages,
    float temperature,
    int max_tokens,
    float frequency_penalty,
    float presence_penalty);

/**
 * @brief Sends an image to the AI for analysis
 * @param messages Vector of Message structures including prompts
 * @param image The image to analyze
 * @param temperature Controls randomness (0.0-1.0)
 * @param max_tokens Maximum tokens to generate
 * @param frequency_penalty Penalizes token frequency (0.0-2.0)
 * @param presence_penalty Penalizes token presence (0.0-2.0)
 * @return AI analysis result as a string
 */
std::string AIImagePrompt(
    const std::vector<Message>& messages,
    const cv::Mat& image,
    float temperature = 0.3f,
    int max_tokens = 1024,
    float frequency_penalty = 0.0f,
    float presence_penalty = 0.0f);

/**
 * @brief Retrieves the OpenAI API key from environment variables
 * @return The API key as a string
 * @throws std::runtime_error if the API key is not set
 */
std::string getApiKey();

} // namespace ai_core

#endif // INSPECT_AI_CORE_HPP