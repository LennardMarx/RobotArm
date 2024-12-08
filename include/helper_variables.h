#ifndef HELPER_VARS_H
#define HELPER_VARS_H

#pragma once
#include "pendulum.h"
#include <array>
#include <vector>

class HelperVars {
public:
  // HelperVars();
  // ~HelperVars();
  //
  // void toggleQuit();
  // bool getQuit();
  //
  // void toggleReset();
  // bool getReset();
  //
  // void toggleTrajOn();
  // bool getTrajOn();
  //
  // std::vector<std::array<double, 2>> &getTrajectory();
  //
  // void toggleController();
  // bool getControllerState();
  //
  // std::array<int, 2> &getRotations();
  void toggleQuit() { quit = !quit; }
  bool getQuit() { return quit; }

  void toggleReset() { reset = !reset; }
  bool getReset() { return reset; }

  void toggleTrajOn() { trajOn = !trajOn; };
  bool getTrajOn() { return trajOn; }

  std::vector<std::array<double, 2>> &getTrajectory() { return trajectory; }

  void toggleController() { controllerState = !controllerState; }
  bool getControllerState() { return controllerState; }

  std::array<int, 2> &getRotations() { return rotations; }

private:
  static inline bool quit = false;
  static inline bool reset = false;
  static inline bool trajOn = true;
  static inline std::vector<std::array<double, 2>> trajectory;
  // 1 = controller on, 0 = off
  static inline bool controllerState = 1;
  static inline std::array<int, 2> rotations = {0, 0};
};

#endif
