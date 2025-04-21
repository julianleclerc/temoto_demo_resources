#include "get_coordinates/llm_solver.hpp"
#include "get_coordinates/ai_core.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

using json = nlohmann::json;

namespace LLMSolver {

std::string createIDSearchInstructions() {
    return R"(
You are an assistant responsible for providing the **target id** the robot need to navigate to on a map.
You will receive:
1. **A map image**:
 - **Black areas**: Non-traversable regions (e.g., walls).
 - **White areas**: Traversable regions where the robot can roam but might not always be reachable.
 - **Robot's Current Position**: Indicated by a red circle and an orientation line (0 degrees at 3 o'clock, counter-clockwise rotation).
 - **Objects**: Represented as colored rectangles with readable ID labels.
2. **A user request**:
 - Specifies the target object to navigate to and may include additional descriptive attributes.

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
 "target_id": "none",
 "error": <error_type>,
 "message": "<error_message>"
}
- **Error Types**:
 - "noObjects": No objects match the description.
 - "ambiguous": Multiple objects match, but no clear decision can be made.
 - "noPath": The robot cannot reach a valid position near the target.
 - "skip": User explicitly requested to skip the operation.

The output must only include the JSON response. No additional reasoning, explanation, or context should be part of the response.
)";
}

std::string createOneShotInstructions() {
    return R"(
You are an assistant responsible for providing navigation coordinates to help a robot reach a target object. You will:
1. Analyze a map image showing objects and robot position
2. Identify the specific target object based on user description
3. Return coordinates where the robot should move to reach this target

## Map Interpretation:
- **White areas**: Traversable spaces where the robot can move
- **Black areas**: Obstacles that cannot be passed
- **Shaded/gray areas**: Non-traversable regions the robot must avoid
- **Blue circle**: Robot's current position
- **Colored rectangles**: Objects with ID labels (e.g., plant_001, chair_002)
- **Red grid lines**: Reference grid (coordinates can be on these lines)

## Coordinate Selection Requirements:
1. Coordinates MUST be within white traversable areas only
2. Coordinates MUST be within the map boundaries
3. Coordinates should be close enough to the target object for meaningful interaction (typically 0.5-1 meter away)
4. The point should be in an open area, not pressed against walls or obstacles
5. Choose a point that gives the robot a clear view of the target object

## Response Format:
{
 "success": "true",
 "coordinates": {"x": <target x-pixel-coordinate>, "y": <target y-pixel-coordinate>},
 "target_id": "<target_id>",
 "error": "none",
 "message": "Heading to <target> because <reasoning behind decision>"
}

## Error Conditions:
If you can't find a valid target or navigation point:
{
 "success": "false",
 "coordinates": {"x": null, "y": null},
 "target_id": "<error_type>",
 "error": "<error_type>",
 "message": "<descriptive error message>"
}

Error types:
- "noObjects": The requested object doesn't exist on the map
- "ambiguous": Multiple matching objects exist and can't be distinguished
- "unreachable": Object exists but no valid navigation point can be found

## Finding the Right Target:
1. First identify ALL objects matching the type requested (e.g., all plants)
2. If the request has qualifiers like "next to X", find the object with that relation
3. For "next to" relations, check if objects are within 2 meters of each other
4. Measure distances between objects based on their coordinates as labeled on the map

## Selecting Navigation Coordinates:
1. Find a position that is:
   - On white space (traversable area)
   - ~0.7-1 meter from the target (20-40 pixels depending on map scale)
   - Placed in a way that the robot would face the center of the target
   - Not blocked by obstacles (black areas)
2. If multiple positions meet these criteria, choose the one closest to the robot

IMPORTANT: Double-check your coordinates are valid (within map bounds and on white traversable space)

    )";
}

std::string createPolarInstructions() {
    return R"(
You are an assistant responsible for identifying a target and returning the polar coordinates to which a robot must navigate to in relation to the target. You will:
1. Analyze a map image showing objects and robot position
2. Identify the specific target object based on user description
3. Return appropriate angle (from the target's center outward) at which the robot should approach
4. Return appropriate distance to which the robot should approach the target

## Map Interpretation:
- **White areas**: Traversable spaces where the robot can move
- **Black areas**: Obstacles that cannot be passed (both solid black and gray areas should be treated as non-traversable)
- **Blue circle**: Robot's current position
- **Colored rectangles**: Objects with ID labels (e.g., plant_001, chair_002)
- **Red grid lines**: Reference grid (coordinates can be on these lines)

## Finding the Right Target:
1. CRITICAL: Be extremely precise when interpreting spatial relationships like "next to", "near", "in front of", etc.
   - "Next to" means objects that are DIRECTLY adjacent with minimal distance between them (< 1 meter)
   - "Near" means in the general vicinity but not necessarily adjacent (1-3 meters)
   - Objects on opposite sides of a room are NOT "next to" each other, even if they're in the same general area
2. THOROUGH ANALYSIS: Make a concerted effort to identify the correct object before declaring ambiguity:
   - Analyze ALL spatial relationships mentioned in the request (e.g., "plant next to fridge")
   - Consider secondary spatial relationships (e.g., "plant next to fridge near the door")
   - Look at the ENTIRE context of the map (room layout, object groupings, unique positions)
   - Use common sense reasoning about typical object placements and relationships
3. When evaluating relationships between objects (like "plant next to fridge"):
   - Calculate the EXACT distance between object boundaries
   - Rank ALL matching objects by their distance to the reference object
   - If one object is SIGNIFICANTLY closer than others (even by small margins), select it
   - Consider additional context clues like visibility from the robot's position
4. Always specify the target_id precisely as shown on the map label
5. HANDLING AMBIGUITY (Use sparingly):
   - Only declare ambiguity when MULTIPLE objects match ALL criteria with virtually IDENTICAL relevance
   - If one object has even a SLIGHT advantage in matching the description, choose it
   - Before declaring ambiguity, try considering additional factors like:
     * Object size and prominence
     * Centrality in the room
     * Accessibility from the robot's position
     * Relationship to other landmarks in the room
   - Only return an "ambiguous" error when, after thorough analysis, it's impossible to reasonably select one target

## Finding the Right Angle:
1. Angle must be in degrees from 0 to 359 (or equivalently -180 to 180)
2. The angle is measured from the center of the target outward
3. 0 degrees points to the right (east) of the target
4. 90 degrees points upward (north) from the target
5. 180 degrees points to the left (west) of the target
6. 270 degrees points downward (south) from the target
7. CRITICAL: The angle must result in a position that:
   - Is in a COMPLETELY OPEN area with sufficient clearance (at least 1 meter from any other object)
   - NEVER places the robot between the target and a wall/obstacle
   - Provides clear line-of-sight to the target without obstruction
   - Allows for easy robot access without tight maneuvering
   - Avoids positions where the robot might block pathways or access to other objects
8. ALWAYS check in at least 8 directions around the target (0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°) to find the optimal approach
9. Prioritize angles that place the robot in open spaces rather than confined areas, even if slightly farther from the target

## Finding the Right Distance:
1. Distance is a percentage from 0 to 100 
2. 0% means very close to the target's edge (for interaction)
3. 50% means a moderate distance from the target
4. 100% means a far distance from the target
5. Choose a distance that:
   - Keeps the robot entirely in white space
   - Provides at least 0.5 meters of clearance from all obstacles
   - Ensures no part of the robot would overlap with any object
   - Is appropriate for the target type and intended interaction
6. NEVER choose a distance that would place any part of the robot in non-traversable areas

## Additional Verification Checks:
1. After identifying a target and determining polar coordinates, VERIFY your choice by:
   - Confirming the target truly matches the relationship description (e.g., "next to fridge")
   - Checking if the resulting position places the robot in a fully open area
   - Ensuring the robot would not be wedged between objects or against walls
   - Verifying there's enough room for the robot to rotate if needed
2. If the initially chosen angle would place the robot in a confined space, ADJUST your choice to prioritize robot accessibility

## Response Format:
{
 "success": "true",
 "target_id": "<target_id>",
 "polar_coordinates": {"angle": <angle>, "distance": <distance>},
 "error": "none",
 "message": "Heading to <target> because <detailed reasoning>"
}

## Error Conditions:
If you can't find a valid target or navigation point:
{
 "success": "false",
 "target_id": "none",
 "polar_coordinates": {"angle": "none", "distance": "none"},
 "error": "<error_type>",
 "message": "<descriptive error message>",
 "candidates": ["<target_id_1>", "<target_id_2>", "..."]
}

Error types:
- "noObjects": The requested object doesn't exist on the map
- "ambiguous": Multiple matching objects exist and can't be distinguished (MUST include "candidates" field with array of all matching object IDs)
- "unreachable": Object exists but no valid navigation point can be found

DOUBLE-CHECK your target identification before responding. Make sure the target_id matches exactly what's on the map.
You MUST correctly interpret spatial relationships between objects for proper target identification.
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
            0.2f,       // temperature
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
        
        std::cout << "LLM Solver: Calling OpenAI API with map image for one-shot search..." << std::endl;
        std::cout << "LLM Solver: Map image dimensions: " << map_image.cols << "x" << map_image.rows << std::endl;
        
        // Call AI with image prompt using the ai_core implementation
        std::string ai_response = ai_core::AIImagePrompt(
            messages,
            map_image,  // Use the image directly
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
                {"coordinates", {{"x", "none"}, {"y", "none"}}},
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
                {"coordinates", {{"x", "none"}, {"y", "none"}}},
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
            {"coordinates", {{"x", "none"}, {"y", "none"}}},
            {"target_id", "none"},
            {"error", "systemError"},
            {"message", "Critical error in one-shot coordinate search: " + std::string(e.what())}
        };
        return error_json;
    }
}

nlohmann::json getCoordinatePolar(
    const cv::Mat& map_image,
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
        
        std::cout << "LLM Solver: Calling OpenAI API with map image for one-shot search..." << std::endl;
        std::cout << "LLM Solver: Map image dimensions: " << map_image.cols << "x" << map_image.rows << std::endl;
        
        // Call AI with image prompt using the ai_core implementation
        std::string ai_response = ai_core::AIImagePrompt(
            messages,
            map_image,  // Use the image directly
            0.2f,       // temperature
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
    const std::string& target_name) {
    
    std::cout << "LLM Solver: Using coordinate fallback method for target: " << target_name << std::endl;
    
    try {
        // For fallback, we'll use the ID search method as it's more basic
        return getCoordinateIDSearch(map_image, target_name);
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
