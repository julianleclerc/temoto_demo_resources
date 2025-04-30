#include "get_coordinates/llm_solver.hpp"
#include "get_coordinates/ai_core.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

using json = nlohmann::json;

namespace LLMSolver {

std::string createOneShotInstructions() {
    return R"(
    You are an assistant responsible for providing the **target id** the robot need to navigate to on a map.

    You will receive:
    1. **A map image**:
       - **Black areas**: Non-traversable regions (e.g., walls).
       - **White areas**: Traversable regions where the robot can roam but might not always be reachable.
       - **Robot's Current Position**: Indicated by a red circle and an orientation line (0 degrees at 3 o'clock, counter-clockwise rotation).
       - **Objects**: Represented as colored rectangles with readable ID labels.
    2. **An object list**:
       - Each entry includes an object's ID, description, attributes (if any), and coordinates.
    3. **A user request**:
       - Specifies the target object to navigate to and may include additional descriptive attributes.
    4. **Conversation history**:
       - If an error occurred previously, this history provides context to assist in decision-making.
    
    ---
    
    ### Key Guidelines:
     **Decision-Making**:
       - Minimize errors by focusing on the map and object list provided.
       - If multiple objects match the description, ALWAYS return ambiguous error unless the user has provided specific distinguishing attributes.
       - Use previous user conversations to clarify intent and improve response accuracy.
       - Use semantic matching for object descriptions - don't require exact matches.
    
    
    ### Workflow:
    
    #### **1. Object Identification**
       - Search the object list for items matching the user's description.
       - Match based on:
         - The object description using semantic matching (not just exact matching)
         - You must use attributes provided (e.g., "next to the fridge").
         - Spatial clues (e.g., proximity, relative position from the robot, what the robot is looking at).
       - Apply flexible matching:
         - "loading area" should match items with "loading" in the description
         - "component storage" should match items with "storing" or "component" in the description
         - Consider synonyms (e.g., "bay" and "area" can be related)
       - If no objects match, set "success": "false" with "error": "noObjects". 
    
    #### **2. Handling Ambiguities**
       - If multiple objects of the same type match (e.g., multiple sofas):
         - DO NOT automatically select the closest one.
         - ONLY resolve ambiguity when user has provided CLEAR distinguishing information such as:
           - Specific size attributes (e.g., "large sofa", "small sofa")
           - Clear positional attributes (e.g., "sofa next to the window", "sofa in the corner")
           - Color or other distinctive properties explicitly mentioned
         - If the user's request lacks these distinguishing details, ALWAYS return "success": "false" with "error": "ambiguous".
         - In the ambiguous error message, provide a list of ALL matching objects with their distinguishing features to help the user clarify.
    
    #### **3. Error Handling**
       - Return an error when:
         - No objects match ("noObjects").
         - Multiple objects of requested type exist without clear distinguishing criteria ("ambiguous").
         - The target is unreachable due to obstacles ("noPath").
       - **Do not attempt to resolve ambiguities automatically** - always ask for clarification when multiple objects of the same type exist.
    
    ---
    
    ### Response Format:
    
    #### **Success Response**:
    If valid coordinates are found:
    {
      "success": "true",
      "target_id": "<target_id>",
      "error": "none",
      "message": "Starting Navigation to <object and description>"
    }
    
    #### **Error Response**:
    If an error occurs:
    {
      "success": "false",
      "target_id": "null",
      "error": "<error_type>",
      "message": "<error_message with detailed information>"
    }
    
    - **Error Types**:
      - "noObjects": No objects match the description.
      - "ambiguous": Multiple objects match, but no clear decision can be made. Include list of all matching objects and their distinguishing features.
      - "noPath": The robot cannot reach a valid position near the target.
      - "skip": User explicitly requested to skip the operation.
    
    ---
    
    ### Example Response for Ambiguity:
    
    #### **User Request**: "Navigate to the sofa."
    
    **Robot's Position**: (x: 100, y: 150, orientation: 0 degrees)  
    **Object List**:
    - sofa_001: (small sofa, near the window)
    - sofa_002: (large sofa, against the wall)
    
    **Logic**:
    1. Identify that both sofa_001 and sofa_002 match the general description "sofa"
    2. Note that the user hasn't specified which sofa they want
    3. Return ambiguity error with details about both sofas
    
    **Response**:
    {
      "success": "false",
      "target_id": "null",
      "error": "ambiguous",
      "message": "Multiple sofas found. Please specify which one: sofa_001 (small sofa near the window) or sofa_002 (large sofa against the wall)."
    }
    
    ---
    
    ### Example Response for Semantic Matching:
    
    #### **User Request**: "Navigate to the loading area."
    
    **Robot's Position**: (x: 100, y: 150, orientation: 0 degrees)  
    **Object List**:
    - area_004: (Loading bay)
    - area_001: (Component storage area)
    
    **Logic**:
    1. Identify that area_004 with description "Loading bay" semantically matches "loading area"
    2. Return success with the appropriate target ID
    
    **Response**:
    {
      "success": "true",
      "target_id": "area_004",
      "error": "none",
      "message": "Starting Navigation to the loading bay (area_004) "
    }
    
    ---
    The output must only include the JSON response. No additional reasoning, explanation, or context should be part of the response. For example:
    {
      "success": "true",
      "target_id": "plant_002",
      "error": "none",
      "message": "Starting Navigation to the green plant on the corner of the room"
    }
    )";
}

std::string createPolarInstructions() {
    return R"(
You are an assistant responsible for identifying a target object and determining the optimal approach for a robot. Your task is to:
1. Analyze the map image showing objects and the robot's current position
2. Identify the specific target object based on the user's description
3. Calculate the angle from robot approach to target and determine the appropriate distance

## CRITICAL: RESPONSE FORMAT REQUIREMENTS
YOU MUST FOLLOW THESE FORMATTING RULES EXACTLY:
1. Your response MUST be a valid JSON object enclosed in curly braces {}
2. Do NOT include any explanatory text, markdown, or code blocks outside the JSON
3. Do NOT include ```json or ``` anywhere in your response
4. Start your response with { and end with } without any additional characters
5. Ensure all JSON keys and string values are enclosed in double quotes

## Map Visual Understanding:
- **White areas**: Open spaces where the robot can move freely
- **Black/Gray areas**: Walls and obstacles the robot cannot pass through
- **Colored rectangles**: Various objects with ID labels
- **Blue circle**: Robot's current position
- **Red Grid**: Is a grid overlay to help in determiing the angle at which the angle at which the robot should approach the target

## Finding the Target Object (Using Visual Assessment):
1. VISUALLY SCAN the map for objects matching the description
2. Look for VISUAL RELATIONSHIPS between objects:
   - "Near the wall" - object visibly close to any black border
   - "In the corner" - object visually located in any corner of the room
   - "Next to the table" - object that appears closest to any table
   - "Between X and Y" - object visually positioned between two other objects
3. Use VISUAL CONTEXT to identify the target, not just label names
4. If multiple objects match, choose the one that BEST FITS the visual description
5. Ensure to examine the relationship of objects both vertically and horizontally (the map is 2d space)

## Determining the Robot-to-Target Angle:
1. Find the target obejct position 
2. Examine the area around
3. Determine the angle of approach from the ROBOT TO THE TARGET in that is the least obscured by obstacles (such as walls or other objects):
   - 0 degrees points to the right (east)
   - 90 degrees points up (north)
   - 180 degrees points to the left (west)
   - 270 degrees points down (south)

## Finding the Right Distance:
1. Distance is a percentage from 0 to 100
2. 0% means very close to the target's edge (for interaction)
3. 50% means a moderate distance from the target
4. 100% means a far distance from the target
5. Choose a distance that:
   - Places the robot in a COMPLETELY WHITE SPACE area
   - Ensures no walls or obstacles are within 10-15 pixels
   - Provides PLENTY OF SPACE for the robot to maneuver
   - Is NOT wedged between the target and a wall or other object

## VISUAL ASSESSMENT CHECKLIST:
- ✓ The chosen distance places the robot approach in a clearly open area
- ✓ No walls or obstacles near the robot's approach position
- ✓ A circle with 10-pixel radius would fit entirely in white space
- ✓ Clear line of sight between robot and target
- ✓ Position allows robot to face the target directly

## WHAT TO AVOID (Visual Red Flags):
- ✕ Positions next to walls or in corners
- ✕ Positions in narrow passages or doorways
- ✕ Placing robot between the target and a wall
- ✕ Cluttered areas with multiple objects nearby

## Example 1:
{
"success": "true",
"target_id": "plant_001",
"distance": 50,
"robot_to_target_angle": 135,
"error": "none",
"message": "Navigating to plant_001. Selected distance provides clear approach with plenty of open space for maneuvering."
}

## Example 2:
{
"success": "false",
"target_id": "none",
"distance": null,
"robot_to_target_angle": null,
"error": "notFound",
"message": "No object matching description found on the map."
}

## Response Format:
{
"success": "true",
"target_id": "<target_id>",
"distance": <distance_percentage>,
"robot_to_target_angle": <angle_in_degrees>,
"error": "none",
"message": "Navigating to <target>. <explanation of selection>"
}

## Error Format:
{
"success": "false",
"target_id": "none",
"distance": null,
"robot_to_target_angle": null,
"error": "<error_type>",
"message": "<descriptive error message>",
"candidates": ["<target_id_1>", "<target_id_2>", "..."]
}

Error types:
- "notFound": The requested object doesn't appear on the map
- "ambiguous": Multiple objects match the description (include all matches in "candidates")
- "noApproach": No good approach position can be found visually

REMEMBER: Your entire response must be valid JSON. Start with { and end with }. No text outside the JSON structure.
)";
}


std::string cleanLLMJsonResponse(const std::string& raw_response) {
    std::cout << "LLM Solver: Cleaning raw LLM response to extract JSON..." << std::endl;
    
    // Find the first opening curly brace
    size_t start_pos = raw_response.find('{');
    if (start_pos == std::string::npos) {
        std::cerr << "LLM Solver: No JSON object found in response (no opening brace)" << std::endl;
        return "";
    }
    
    // Find the last closing curly brace
    size_t end_pos = raw_response.rfind('}');
    if (end_pos == std::string::npos) {
        std::cerr << "LLM Solver: No JSON object found in response (no closing brace)" << std::endl;
        return "";
    }
    
    // Extract just the JSON object
    if (end_pos <= start_pos) {
        std::cerr << "LLM Solver: Invalid JSON structure (closing brace before opening brace)" << std::endl;
        return "";
    }
    
    std::string cleaned_json = raw_response.substr(start_pos, end_pos - start_pos + 1);
    std::cout << "LLM Solver: Extracted JSON: " << (cleaned_json.length() > 100 ? 
        (cleaned_json.substr(0, 97) + "...") : cleaned_json) << std::endl;
    
    return cleaned_json;
}

nlohmann::json getCoordinateIDSearch(
    const cv::Mat& map_image,
    const nlohmann::json& items_data,
    const std::string& target_name) {
    
    std::cout << "LLM Solver: Starting ID search for target: " << target_name << std::endl;
    
    try {
        // Prepare the system instructions
        std::string system_instructions = createIDSearchInstructions();
        
        // Create messages for the AI
        std::vector<ai_core::Message> messages;
        
        // System message to define the AI's role
        ai_core::Message system_message;
        system_message.role = "system";
        system_message.content = system_instructions;
        messages.push_back(system_message);
        
        // Create the user prompt with the target information
        std::string user_message = "I need to find the following target on the map: " + target_name;
        
        // Create user message object
        ai_core::Message user_msg;
        user_msg.role = "user";
        user_msg.content = user_message;
        messages.push_back(user_msg);
        
        std::cout << "LLM Solver: Calling OpenAI API with map image for ID search..." << std::endl;
        std::cout << "LLM Solver: Map image dimensions: " << map_image.cols << "x" << map_image.rows << std::endl;
        
        // Call AI with image prompt using the ai_core implementation
        std::string ai_response = ai_core::AIImagePrompt(
            messages,
            map_image,  
            0.4f,       // temperature
            1024,       // max_tokens
            0.0f,       // frequency_penalty
            0.0f        // presence_penalty
        );
        
        // Clean the response to extract valid JSON
        std::string cleaned_response = cleanLLMJsonResponse(ai_response);
        if (cleaned_response.empty()) {
            std::cerr << "LLM Solver: Failed to extract valid JSON from LLM response" << std::endl;
            
            // Return error response for processing failure
            json error_json = {
                {"success", "false"},
                {"target_id", "none"},
                {"error", "systemError"},
                {"message", "Failed to extract valid JSON from LLM response"}
            };
            return error_json;
        }
        
        // Parse the cleaned response
        try {
            json response_json = json::parse(cleaned_response);
            std::cout << "LLM Solver: Successfully parsed JSON response for ID search" << std::endl;            
            return response_json;
        } catch (const json::exception& e) {
            std::cerr << "LLM Solver: JSON parsing error: " << e.what() << std::endl;
            
            // Return error response for JSON parsing failure
            json error_json = {
                {"success", "false"},
                {"target_id", "none"},
                {"error", "systemError"},
                {"message", "Failed to parse LLM response: " + std::string(e.what())}
            };
            return error_json;
        }
    } catch (const std::exception& e) {
        std::cerr << "LLM Solver: Critical error in getCoordinateIDSearch: " << e.what() << std::endl;
        
        // Return error response for any critical failure
        json error_json = {
            {"success", "false"},
            {"target_id", "none"},
            {"error", "systemError"},
            {"message", "Critical error in coordinate ID search: " + std::string(e.what())}
        };
        return error_json;
    }
}

nlohmann::json getCoordinateOneShot(
    const cv::Mat& map_image,
    const nlohmann::json& items_data,
    const std::string& target_name) {
    
    std::cout << "LLM Solver: Starting one-shot coordinate search for target: " << target_name << std::endl;
    
    try {
        // Prepare the system instructions
        std::string system_instructions = createOneShotInstructions();
        
        // Create messages for the AI
        std::vector<ai_core::Message> messages;
        
        // System message to define the AI's role
        ai_core::Message system_message;
        system_message.role = "system";
        system_message.content = system_instructions;
        messages.push_back(system_message);
        
        // Create the user prompt with the target information
        std::string user_message = "Desired Target: " + target_name;
        
        // Create user message object
        ai_core::Message user_msg;
        user_msg.role = "user";
        user_msg.content = user_message;
        messages.push_back(user_msg);

        // Add items
        ai_core::Message items_msg;
        items_msg.role = "user";
        items_msg.content = "Available items:\n" + items_data.dump(2);
        messages.push_back(items_msg);

        std::cout << "LLM Solver: Calling OpenAI API with map image for one-shot search..." << std::endl;
        std::cout << "LLM Solver: Map image dimensions: " << map_image.cols << "x" << map_image.rows << std::endl;
        
        // Call AI with image prompt using the ai_core implementation
        std::string ai_response = ai_core::callOpenAIAPI(
            messages,
            0.3f,       // temperature
            1024,       // max_tokens
            0.0f,       // frequency_penalty
            0.0f        // presence_penalty
        );
        
        // Clean the response to extract valid JSON
        std::string cleaned_response = cleanLLMJsonResponse(ai_response);
        if (cleaned_response.empty()) {
            std::cerr << "LLM Solver: Failed to extract valid JSON from LLM response" << std::endl;
            
            // Return error response for processing failure
            json error_json = {
                {"success", "false"},
                {"target_id", "none"},
                {"error", "systemError"},
                {"message", "Failed to extract valid JSON from LLM response"}
            };
            return error_json;
        }
        
        // Parse the cleaned response
        try {
            json response_json = json::parse(cleaned_response);
            std::cout << "LLM Solver: Successfully parsed JSON response for one-shot search" << std::endl;
            return response_json;
        } catch (const json::exception& e) {
            std::cerr << "LLM Solver: JSON parsing error: " << e.what() << std::endl;
            
            // Return error response for JSON parsing failure
            json error_json = {
                {"success", "false"},
                {"target_id", "none"},
                {"error", "systemError"},
                {"message", "Failed to parse LLM response: " + std::string(e.what())}
            };
            return error_json;
        }
    } catch (const std::exception& e) {
        std::cerr << "LLM Solver: Critical error in getCoordinateOneShot: " << e.what() << std::endl;
        
        // Return error response for any critical failure
        json error_json = {
            {"success", "false"},
            {"target_id", "none"},
            {"error", "systemError"},
            {"message", "Critical error in one-shot coordinate search: " + std::string(e.what())}
        };
        return error_json;
    }
}

nlohmann::json getCoordinatePolar(
    const cv::Mat& map_image,
    const nlohmann::json& items_data,
    const std::string& target_name) {
    
    std::cout << "LLM Solver: Starting polar coordinate search for target: " << target_name << std::endl;
    
    try {
        // Prepare the system instructions
        std::string system_instructions = createPolarInstructions();
        
        // Create messages for the AI
        std::vector<ai_core::Message> messages;
        
        // System message to define the AI's role
        ai_core::Message system_message;
        system_message.role = "system";
        system_message.content = system_instructions;
        messages.push_back(system_message);
        
        // Create the user prompt with the target information
        std::string user_message = "Desired Target: " + target_name;
        
        // Create user message object
        ai_core::Message user_msg;
        user_msg.role = "user";
        user_msg.content = user_message;
        messages.push_back(user_msg);
        
        // Add items
        ai_core::Message items_msg;
        items_msg.role = "user";
        items_msg.content = "Available items:\n" + items_data.dump(2);
        messages.push_back(items_msg);


        std::cout << "LLM Solver: Calling OpenAI API with map image for one-shot search..." << std::endl;
        std::cout << "LLM Solver: Map image dimensions: " << map_image.cols << "x" << map_image.rows << std::endl;
        
        // Call AI with image prompt using the ai_core implementation
        std::string ai_response = ai_core::AIImagePrompt(
            messages,
            map_image,  // Use the image directly
            0.6f,       // temperature
            1024,       // max_tokens
            0.0f,       // frequency_penalty
            0.0f        // presence_penalty
        );
        
        // Clean the response to extract valid JSON
        std::string cleaned_response = cleanLLMJsonResponse(ai_response);
        if (cleaned_response.empty()) {
            std::cerr << "LLM Solver: Failed to extract valid JSON from LLM response" << std::endl;
            
            // Return error response for processing failure
            json error_json = {
                {"success", "false"},
                {"polar_coordinates", {{"x", "none"}, {"y", "none"}}},
                {"target_id", "none"},
                {"error", "systemError"},
                {"message", "Failed to extract valid JSON from LLM response"}
            };
            return error_json;
        }
        
        // Parse the cleaned response
        try {
            json response_json = json::parse(cleaned_response);
            std::cout << "LLM Solver: Successfully parsed JSON response for polar search" << std::endl;
            return response_json;
        } catch (const json::exception& e) {
            std::cerr << "LLM Solver: JSON parsing error: " << e.what() << std::endl;
            
            // Return error response for JSON parsing failure
            json error_json = {
                {"success", "false"},
                {"polar_coordinates", {{"x", "none"}, {"y", "none"}}},
                {"target_id", "none"},
                {"error", "systemError"},
                {"message", "Failed to parse LLM response: " + std::string(e.what())}
            };
            return error_json;
        }
    } catch (const std::exception& e) {
        std::cerr << "LLM Solver: Critical error in getCoordinatePolar: " << e.what() << std::endl;
        
        // Return error response for any critical failure
        json error_json = {
            {"success", "false"},
            {"polar_coordinates", {{"x", "none"}, {"y", "none"}}},
            {"target_id", "none"},
            {"error", "systemError"},
            {"message", "Critical error in polar coordinate search: " + std::string(e.what())}
        };
        return error_json;
    }
}


nlohmann::json getCoordinateFallback(
    const cv::Mat& map_image,
    const nlohmann::json& items_data,
    const std::string& target_name) {
    
    std::cout << "LLM Solver: Using coordinate fallback method for target: " << target_name << std::endl;
    
    try {
        // For fallback, we'll use the ID search method as it's more basic
        return getCoordinateIDSearch(map_image, items_data, target_name);
    } catch (const std::exception& e) {
        std::cerr << "LLM Solver: Critical error in getCoordinateFallback: " << e.what() << std::endl;
        
        // Return a default response on any failure
        json default_json = {
            {"success", "true"},
            {"target_id", "fallback_target_001"},
            {"error", "none"},
            {"message", "Fallback navigation to target: " + target_name}
        };
        return default_json;
    }
}

} // namespace LLMSolver
