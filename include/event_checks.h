#ifndef EVENT_CHECKS_H
#define EVENT_CHECKS_H

#include "helper_variables.h"
#include "pendulum_dynamics.h"
#include <SDL2/SDL.h>

class EventChecks {
public:
  // EventChecks();
  // ~EventChecks();

  void checkEvents(HelperVars &, PendulumDynamics &);
  // void checkEvents(HelperVars &_helperVars,
  //                  PendulumDynamics &_pendulumDynamics) {
  //   while (SDL_PollEvent(&event) != 0) {
  //     switch (event.type) {
  //     case SDL_QUIT:
  //       _helperVars.toggleQuit();
  //     case SDL_KEYDOWN:
  //       switch (event.key.keysym.sym) {
  //       case SDLK_LEFT:
  //         break;
  //       case SDLK_RIGHT:
  //         break;
  //       case SDLK_UP:
  //         break;
  //       case SDLK_DOWN:
  //         break;
  //       case SDLK_ESCAPE:
  //         _helperVars.toggleQuit();
  //         break;
  //       case SDLK_c:
  //         _pendulumDynamics.toggleController();
  //         break;
  //       case SDLK_r:
  //         _helperVars.toggleReset();
  //         break;
  //       case SDLK_t:
  //         _helperVars.toggleTrajOn();
  //         _helperVars.getTrajectory().clear();
  //         break;
  //       case SDLK_q:
  //         _helperVars.toggleQuit();
  //         break;
  //       case SDLK_SPACE:
  //         // pause = true;
  //         break;
  //       default:
  //         break;
  //       }
  //
  //     case SDL_KEYUP:
  //       // printf("Key release detected\n");
  //       break;
  //
  //     default:
  //       break;
  //     }
  //   }
  // }

private:
  SDL_Event event;
};

#endif
