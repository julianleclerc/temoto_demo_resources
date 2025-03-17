#include "inspect/temoto_action.hpp"

class Inspect : public TemotoAction
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * REQUIRED class methods, do not remove them
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

Inspect()
{
}

bool onRun()
{
  TEMOTO_PRINT_OF("Running", getName());

  /*
   * Implement your code here
   *
   * Access input parameters via "params_in" member
   * Set output parameters via "params_out" member
   */

  return true;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * OPTIONAL class methods, can be removed if not needed
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());
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

~Inspect()
{
}

}; // Inspect class

// REQUIRED, do not remove
boost::shared_ptr<ActionBase> factory()
{
    return boost::shared_ptr<Inspect>(new Inspect());
}

// REQUIRED, do not remove
BOOST_DLL_ALIAS(factory, Inspect)