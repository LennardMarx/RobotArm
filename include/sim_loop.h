#ifndef SIM_LOOP_H
#define SIM_LOOP_H

#pragma once
#include "UI.h"
#include "event_checks.h"
#include "helper_variables.h"
#include "pendulum.h"
#include "pendulum_dynamics.h"
#include <cstdlib>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include <SDL2/SDL.h>
#include <ctime>
#include <unistd.h>
#include <vector>

struct context {
  const int FPS = 60;                // set FPS
  const int frameDelay = 1000 / FPS; // delay according to FPS
  Uint32 frameStart;                 // keeps track of time (?)
  int frameTime;

  int window_width = 1200;
  int window_height = 800;
  UI ui{window_width, window_height};

  Pendulum pendulum{0, 0};
  PendulumDynamics pendulumDynamics;
  HelperVars helperVars;
  EventChecks eventChecks;
  Uint32 mouseState;

  double pi = 3.141592653589793238462643383279502884197;
  double x0 = 0, y0 = 0, x1, y1, x2, y2; // link positions
  double x2_prev, y2_prev;               // previous endeffector position
  double l1 = 150, l2 = 150;             // pendulum length on screen

  int x, y;
  double x_conv;
  double y_conv;

  double qd1 = 0; // desired angle 1
  double qd2 = 0; // desired angle 2

  double xd = 0;  // desired x (when doing inverse kinematics) (!! in meters not
                  // pixels !!)
  double yd = -1; // desired y

  bool pause = false;
  bool controllerOff = false;

  int frameCount = 0;
};

class SimLoop {
public:
  SimLoop();
  ~SimLoop();

  // Pendulum pendulum;
  context ctx;

  void run();

private:
  static void mainloop(void *arg);

private:
};

#endif
