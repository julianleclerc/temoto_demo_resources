// Simplified get_coordinates.cpp
#include "get_coordinates/temoto_action.hpp"
#include <fmt/core.h>
#include <chrono>
#include <thread>
#include <iostream>

#include <nlohmann/json.hpp>

using json = nlohmann::json;


class GetCoordinates : public TemotoAction
{
public:
    GetCoordinates() // REQUIRED
    {
        // Empty constructor
    }

    void onInit()
    {
        TEMOTO_PRINT_OF("Initializing", getName());
        std::cout << "GetCoordinates: onInit called for " << getName() << std::endl;
    }

    bool onRun() // REQUIRED
    {
        //json temoto_log;
        //temoto_log["type"] = "error";
        //temoto_log["message"] = "multiple plants present, please be more precice";
        //writeLog(temoto_log.dump());
        //throw std::runtime_error("Potential issue raised in the inspection");

        try {
            std::cout << "GetCoordinates: onRun called for " << getName() << std::endl;
            std::string output = fmt::format("Getting coordinates for: {}", params_in.target);
            TEMOTO_PRINT_OF(output, getName());
            
        

            // Simple pause
            uint sleep_ms{1000};
            for (uint i{0}; i<2 && actionOk(); i++)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
                std::cout << ". " << std::flush;
            }
            std::cout << std::endl;
            
            // Set fixed output values
            params_out.pose.position.x = 1.0;
            params_out.pose.position.y = 2.0;
            params_out.pose.position.z = 0.0;
            params_out.pose.orientation.r = 0.0;
            params_out.pose.orientation.p = 0.0;
            params_out.pose.orientation.y = 0.0;
            
            TEMOTO_PRINT_OF("Done\n", getName());
            return true;
        } catch (const std::exception& e) {
            std::cerr << "GetCoordinates: Error in onRun: " << e.what() << std::endl;
            return false;
        }
    }

    void onPause()
    {
        TEMOTO_PRINT_OF("Pausing", getName());
    }

    void onResume()
    {
        TEMOTO_PRINT_OF("Continuing", getName());
    }

    void onStop()
    {
        TEMOTO_PRINT_OF("Stopping", getName());
    }

    ~GetCoordinates()
    {
        TEMOTO_PRINT_OF("Destroying", getName());
    }
}; // GetCoordinates class

// REQUIRED, do not remove
boost::shared_ptr<ActionBase> factory()
{
    return boost::shared_ptr<GetCoordinates>(new GetCoordinates());
}

// REQUIRED, do not remove
BOOST_DLL_ALIAS(factory, GetCoordinates)