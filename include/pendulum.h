#ifndef PENDULUM_H
#define PENDULUM_H

#pragma once
#include "pendulum_dynamics.h"
#include <array>

class Pendulum {
public:
  Pendulum(double, double); // constructor
  virtual ~Pendulum();      // destructor

  // get and set methods
  std::array<double, 4> &getStates();
  void setStates(std::array<double, 4>);

  bool getReset();
  void setReset(bool);

  std::array<int, 2> getRotations();
  void updateRotations();

  std::array<double, 4> &getPreviousStates();
  void keepBetweenZeroAndPi();
  // Pendulum(double _th1, double _th2)
  //     : pendulumStates({_th1, _th2, 0, 0}) {} // constructor
  // ~Pendulum() {}                              // destructor
  //
  // std::array<double, 4> &getStates() { return pendulumStates; }
  // void setStates(std::array<double, 4> _x) { pendulumStates = _x; }
  //
  // bool getReset() { return reset; }
  // void setReset(bool _reset) { reset = _reset; }
  //
  // std::array<int, 2> getRotations() {
  //   updateRotations();
  //   return rotations;
  // }
  //
  // void updateRotations() {
  //   rotations[0] = getStates()[0] / (2 * pi);
  //   rotations[1] = getStates()[1] / (2 * pi);
  // }
  //
  // std::array<double, 4> &getPreviousStates() { return previousStates; }
  // void keepBetweenZeroAndPi() {
  //   if (getStates()[0] >= 2 * pi && getPreviousStates()[0] < 2 * pi) {
  //     getStates()[0] -= 2 * pi;
  //   }
  //   if (getStates()[0] < 0 && getPreviousStates()[0] >= 0) {
  //     getStates()[0] += 2 * pi;
  //   }
  // }

private:
  const double pi = 3.141592653589793238462643383279502884197;
  bool reset = false;
  std::array<double, 4> pendulumStates; // drone states
  std::array<double, 4> previousStates;
  std::array<int, 2> rotations = {0, 0};

public:
  // variables of the robot arm
  // static -> immutable?
  // inline -> would else have to be declared (?) in .ccp
  static inline const double beta1 = 0.02; // Damping on joint 1
  static inline const double beta2 = 0.02; // Damping on joint 2
  static inline const double m1 = 1;       // Mass Link 1
  static inline const double m2 = 1;       // Mass Link 2
  static inline const double l1 = 0.5;     // Length Link 1
  static inline const double l2 = 0.5;     // Length Link 2
};

#endif
