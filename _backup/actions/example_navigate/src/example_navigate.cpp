#include "example_navigate/temoto_action.hpp"

#include <fmt/core.h>
#include <chrono>
#include <thread>

class ExampleNavigate : public TemotoAction
{
public:

ExampleNavigate() // REQUIRED
{
}

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());
}

bool onRun() // REQUIRED
{

  std::string output = fmt::format("Moving to '{}' \nx = {:<10} r = {}\ny = {:<10} p = {}\nz = {:<10} y = {}",
    params_in.location,
    params_in.pose.position.x, params_in.pose.orientation.r,
    params_in.pose.position.y, params_in.pose.orientation.p,
    params_in.pose.position.z, params_in.pose.orientation.y);

  TEMOTO_PRINT_OF(output, getName());

  uint sleep_ms{1000};
  for (uint i{0}; i<3 && actionOk() ; i++)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    std::cout << ". " << std::flush;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  std::cout << std::endl;

  TEMOTO_PRINT_OF("Done\n", getName());

  return true;
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

~ExampleNavigate()
{
}

}; // ExampleNavigate class

boost::shared_ptr<ActionBase> factory()
{
    return boost::shared_ptr<ExampleNavigate>(new ExampleNavigate());
}

BOOST_DLL_ALIAS(factory, ExampleNavigate)
