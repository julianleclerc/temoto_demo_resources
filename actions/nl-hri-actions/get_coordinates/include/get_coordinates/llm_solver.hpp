#ifndef LLM_SOLVER_HPP
#define LLM_SOLVER_HPP

#include <string>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

namespace LLMSolver {

/**
 * @brief ID Search method for getting coordinates
 * 
 * @param map_image The map image as OpenCV Mat
 * @param target_name The name of the target
 * @return nlohmann::json Response with the target information
 */
nlohmann::json getCoordinateIDSearch(
    const cv::Mat& map_image,
    const nlohmann::json& items_data,
    const std::string& target_name);

/**
 * @brief One-shot method for getting coordinates
 * 
 * @param map_image The map image as OpenCV Mat
 * @param target_name The name of the target
 * @return nlohmann::json Response with the target information
 */
nlohmann::json getCoordinateOneShot(
    const cv::Mat& map_image,
    const nlohmann::json& items_data,
    const std::string& target_name);

/**
 * @brief Polar method for getting coordinates
 * 
 * @param map_image The map image as OpenCV Mat
 * @param target_name The name of the target
 * @return nlohmann::json Response with the target information
 */
nlohmann::json getCoordinatePolar(
    const cv::Mat& map_image,
    const nlohmann::json& items_data, 
    const std::string& target_name);

/**
 * @brief Fallback method for getting coordinates when full implementation isn't available
 * 
 * @param map_image The map image as OpenCV Mat
 * @param target_name The name of the target
 * @return nlohmann::json Response with the target information
 */
nlohmann::json getCoordinateFallback(
    const cv::Mat& map_image,
    const nlohmann::json& items_data,
    const std::string& target_name);

/**
 * @brief Create the system instructions for the ID Search method
 * 
 * @return std::string The formatted system instructions
 */
std::string createIDSearchInstructions();

/**
 * @brief Create the system instructions for the One-shot method
 * 
 * @return std::string The formatted system instructions
 */
std::string createOneShotInstructions();

/**
 * @brief Clean JSON response from LLM to ensure valid JSON format
 * 
 * @param raw_response The raw response from the LLM
 * @return std::string Cleaned JSON string
 */
std::string cleanLLMJsonResponse(const std::string& raw_response);

} // namespace LLMSolver

#endif // LLM_SOLVER_HPP