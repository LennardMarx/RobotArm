#ifndef EVENT_CHECKS_H
#define EVENT_CHECKS_H

#include "../include/pendulum_dynamics.h"
#include <SDL2/SDL.h>

class HelperVars
{
public:
    HelperVars();
    ~HelperVars();

    void toggleQuit();
    bool getQuit();

    void toggleReset();
    bool getReset();
private:
    static inline bool quit = false;
    static inline bool reset = false;
};

//======================================================================

class EventChecks
{
public:
    EventChecks();
    ~EventChecks();

    void checkEvents(HelperVars&, PendulumDynamics&);
private:
    SDL_Event event;
};

#endif