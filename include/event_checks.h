#ifndef EVENT_CHECKS_H
#define EVENT_CHECKS_H

#include "../include/helper_variables.h"
#include "../include/pendulum_dynamics.h"
#include <SDL2/SDL.h>

class EventChecks {
public:
  EventChecks();
  ~EventChecks();

  void checkEvents(HelperVars &, PendulumDynamics &);

private:
  SDL_Event event;
};

#endif
