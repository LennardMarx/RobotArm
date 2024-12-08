#ifndef PENDULUM_DYNAMICS_H
#define PENDULUM_DYNAMICS_H

#pragma once
#include "helper_variables.h"
#include "pendulum.h"
#include <array>
#include <cmath>
#include <vector>
// #include <cstdio>
// #include <fstream>
#include <iostream>

class PendulumDynamics {
public:
  PendulumDynamics();          // contructor
  virtual ~PendulumDynamics(); // destructor

  // get and set methods
  bool getReset();
  void setReset(bool);

  std::array<double, 2> getReceivedInputs();
  void setReceivedInputs(std::array<double, 2>);

  std::array<double, 4> getReceivedStates();
  void setReceivedStates(const std::array<double, 4> &);

  std::array<double, 4> getUpdatedStates();
  void setUpdatedStates(std::array<double, 4>);

  std::array<double, 2> inverseKinematics(std::array<double, 2>);

  void toggleController();

  // virtual runge kutta methods to be overridden
  void rungeKutta();
  virtual std::array<double, 4> f(std::array<double, 4>,
                                  std::array<double, 2> &);
  std::array<double, 4> addArrays(std::array<double, 4>, std::array<double, 4>,
                                  double);

  std::array<int, 2> getRotations();
  void updateRotations();

  void print();

  // PendulumDynamics() // constructor
  // {
  //   // initial states
  //   setUpdatedStates({0, 0, 0, 0});
  //   setReceivedStates({0, 0, 0, 0});
  //   setReceivedInputs({0, 0});
  // }
  // virtual ~PendulumDynamics() {} // destructorv

  // variables of the pendulum/robot arm
  // double m1 = Pendulum::m1;
  // double m2 = Pendulum::m2;
  // double l1 = Pendulum::l1;
  // double l2 = Pendulum::l2;
  // double beta1 = Pendulum::beta1;
  // double beta2 = Pendulum::beta2;
  static inline const double beta1 = 0.02; // Damping on joint 1
  static inline const double beta2 = 0.02; // Damping on joint 2
  static inline const double m1 = 1;       // Mass Link 1
  static inline const double m2 = 1;       // Mass Link 2
  static inline const double l1 = 0.5;     // Length Link 1
  static inline const double l2 = 0.5;     // Length Link 2

  // get and set functions
  // bool getReset() { return reset; }
  //
  // void setReset(bool _setup) { reset = _setup; }
  //
  // std::array<double, 2> getReceivedInputs() { return receivedInputs; }
  // void setReceivedInputs(std::array<double, 2> _u) { receivedInputs = _u; }
  //
  // std::array<double, 4> getReceivedStates() { return receivedStates; }
  // void setReceivedStates(const std::array<double, 4> &_states) {
  //   receivedStates = _states;
  // }
  //
  // std::array<double, 4> getUpdatedStates() { return updatedStates; }
  // void setUpdatedStates(std::array<double, 4> _xdot) { updatedStates = _xdot;
  // }
  //
  // std::array<int, 2> getRotations() {
  //   updateRotations();
  //   return rotations;
  // }
  // void updateRotations() {
  //   rotations[0] = getReceivedStates()[0] / (2 * pi);
  //   rotations[1] = getReceivedStates()[1] / (2 * pi);
  // }
  //
  // // runge kutta method calculating the intermediate steps
  // void rungeKutta() {
  //   std::array<double, 4> _updatedStates;
  //   K1 = f(receivedStates, receivedInputs);
  //   K2 = f(addArrays(receivedStates, K1, 2), receivedInputs);
  //   K3 = f(addArrays(receivedStates, K2, 2), receivedInputs);
  //   K4 = f(addArrays(receivedStates, K3, 1), receivedInputs);
  //   for (int i = 0; i < 4; ++i) {
  //     _updatedStates[i] = receivedStates[i] +
  //                         (h * ((K1[i] + 2 * K2[i] + 2 * K3[i] + K4[i]) /
  //                         6));
  //   }
  //   // account for rotations
  //   //  print();
  //   //  _updatedStates[0] -= ((int)(receivedStates[0] / (2 * pi)) -
  //   //  rotation_reference + 1) * 2 * pi; _updatedStates[1] -=
  //   getRotations()[1]
  //   //  * 2 * pi;
  //   setUpdatedStates(_updatedStates);
  // }
  //
  // void print() {
  //   updateRotations();
  //   // std::cout << rotation_reference << ", " << rotations[0] << ", " <<
  //   // rotations[1] << std::endl;
  //   std::cout << (int)(receivedStates[0] / (2 * pi)) << ", "
  //             << (int)(receivedStates[1] / (2 * pi)) << ", "
  //             << rotation_reference[0] << std::endl;
  // }
  //
  // // the method to integrate the runge kutta intermediate steps
  // // dynamics of motion (matrix form, relative _angles, damping)
  // std::array<double, 4> f(std::array<double, 4> _states,
  //                         std::array<double, 2> &_u) {
  //   std::array<double, 4> _output;
  //   // theta1_dot
  //   _output[0] = _states[2];
  //   // theta2_dot
  //   _output[1] = _states[3];
  //
  //   // Inertia determinant
  //   det_M = (((m1 + m2) * l1 * l1 + m2 * l2 * l2 +
  //             2 * m2 * l1 * l2 * cos(_states[1])) *
  //            m2 * l2 * l2) -
  //           ((m2 * l2 * l2 + m2 * l1 * l2 * cos(_states[1])) *
  //            (m2 * l2 * l2 + m2 * l1 * l2 * cos(_states[1])));
  //
  //   // Coriolis forces
  //   w1_C = (-m2 * l1 * l2 * (2 * _states[2] + _states[3]) * sin(_states[1]))
  //   *
  //          _states[3];
  //   w2_C = (m2 * l1 * l2 * _states[2] * sin(_states[1])) * _states[2];
  //
  //   // Gravity terms
  //   w1_G = ((m1 + m2) * l1 * sin(_states[0]) +
  //           m2 * l2 * sin(_states[0] + _states[1])) *
  //          g;
  //   w2_G = (m2 * l2 * sin(_states[0] + _states[1])) * g;
  //
  //   // Damping
  //   w1_D = beta1 * _states[2];
  //   w2_D = beta2 * _states[3];
  //
  //   if (controllerState) {
  //     // Control Law / Input, kp, kd, gravity compensation
  //     tau1 = kp * (_u[0] - _states[0]) - kd * (_states[2]) +
  //            ((m1 + m2) * l1 * sin(_u[0]) + m2 * l2 * sin(_u[0] + _u[1])) *
  //            g;
  //     tau2 = kp * (_u[1] - _states[1]) - kd * (_states[3]) +
  //            (m2 * l2 * sin(_u[0] + _u[1])) * g;
  //
  //     // define max motor strength
  //     if (tau1 > tau1_limit) {
  //       tau1 = tau1_limit;
  //     }
  //     if (tau1 < -tau1_limit) {
  //       tau1 = -tau1_limit;
  //     }
  //     if (tau2 > tau2_limit) {
  //       tau2 = tau2_limit;
  //     }
  //     if (tau2 < -tau2_limit) {
  //       tau2 = -tau2_limit;
  //     }
  //   }
  //   // when controller is switched off
  //   else if (!controllerState) {
  //     tau1 = 0;
  //     tau2 = 0;
  //   }
  //   // terms to be multiplied with M_inv matrix
  //   term1 = tau1 - w1_C - w1_G - w1_D;
  //   term2 = tau2 - w2_C - w2_G - w2_D;
  //
  //   // anglular accelerations omega1_dot and omega2_dot
  //   _output[2] =
  //       ((m2 * l2 * l2) / det_M * term1 +
  //        (-m2 * l2 * l2 - m2 * l1 * l2 * cos(_states[1])) / det_M * term2);
  //   _output[3] =
  //       (-m2 * l2 * l2 - m2 * l1 * l2 * cos(_states[1])) / det_M * term1 +
  //       ((m1 + m2) * l1 * l1 + m2 * l2 * l2 +
  //        2 * m2 * l1 * l2 * cos(_states[1])) /
  //           det_M * term2;
  //
  //   return _output;
  // }
  //
  // std::array<double, 2> inverseKinematics(std::array<double, 2> _pos) {
  //   // double l1 = Pendulum::l1;
  //   // double l2 = Pendulum::l2;
  //   double l1 = 0.5; // Length Link 1
  //   double l2 = 0.5; // Length Link 2
  //
  //   std::array<double, 2> _angles;
  //   double x = _pos[0];
  //   double y = _pos[1];
  //   double x0, y0;
  //
  //   // counting turn if endeffector crosses origin (+- 2 pi)
  //   if (x < 0 && x_prev >= 0 && y < 0) {
  //     rotation_reference[0] -= 1;
  //   } else if (x >= 0 && x_prev < 0 && y < 0) {
  //     rotation_reference[0] += 1;
  //   }
  //   // saving previous x position
  //   x_prev = x;
  //
  //   // calculating inverse kinematics always in quadtrant 1 and transforming
  //   to
  //   // other quadrants!
  //   if (x >= 0 && y >= 0) {
  //     _angles[1] = acos((x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2));
  //     _angles[0] =
  //         atan(y / x) -
  //         atan((l2 * sin(_angles.at(1))) / (l1 + l2 * cos(_angles.at(1)))) +
  //         pi / 2;
  //   } else if (x < 0 && y < 0) {
  //     _angles[1] = acos((x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2));
  //     _angles[0] =
  //         pi + atan(y / x) -
  //         atan((l2 * sin(_angles.at(1))) / (l1 + l2 * cos(_angles.at(1)))) +
  //         pi / 2;
  //   } else if (x >= 0 && y < 0) {
  //     x0 = -y;
  //     y0 = x;
  //     y = y0;
  //     x = x0; // flipping coordinates for quad. 4
  //     _angles[1] = acos((x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2));
  //     _angles[0] =
  //         -pi / 2 + atan(y / x) -
  //         atan((l2 * sin(_angles.at(1))) / (l1 + l2 * cos(_angles.at(1)))) +
  //         pi / 2;
  //   } else if (x < 0 && y >= 0) {
  //     x0 = y;
  //     y0 = -x;
  //     y = y0;
  //     x = x0; // flipping coordinates for quad. 2
  //     _angles[1] = acos((x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2));
  //     _angles[0] =
  //         pi / 2 + atan(y / x) -
  //         atan((l2 * sin(_angles.at(1))) / (l1 + l2 * cos(_angles.at(1)))) +
  //         pi / 2;
  //   }
  //
  //   // correct for discrepency between rotations of pendulum and reference
  //   if (!controllerState) {
  //     rotation_reference[0] += rotation_reference[0] - getRotations()[0];
  //     rotation_reference[1] += rotation_reference[1] - getRotations()[1];
  //   }
  //
  //   _angles[0] += rotation_reference[0] * 2 * pi;
  //   _angles[1] += rotation_reference[1] * 2 * pi;
  //   // std::cout << _angles[0] << std::endl;
  //
  //   return _angles;
  // }
  //
  // void toggleController() { controllerState = !controllerState; }
  //
  // // method to add each position of an array with the according position of
  // // another _over to account for the multiplication with h/2 or h in the
  // runge
  // // kutta steps
  // std::array<double, 4> addArrays(std::array<double, 4> _x,
  //                                 std::array<double, 4> _K, double _over) {
  //   std::array<double, 4> addedArray;
  //   for (int i = 0; i < 4; ++i) {
  //     addedArray.at(i) = _x.at(i) + _K.at(i) * h / _over;
  //   }
  //   return addedArray;
  // }

private:            // maybe change to private -> pass by reference
  double h = 0.001; // Time step
  bool reset = false;
  std::array<int, 2> rotations = {0, 0};

  std::array<double, 4> receivedStates; // array to save received drone states
  std::array<double, 2> receivedInputs; // array to save reveiced inputs
  std::array<double, 4> updatedStates;  // calculated new drone states
  std::array<double, 4> K1, K2, K3, K4; // RK4 intermediate steps

  const double g = 9.81; // Gravity
  const double pi = 3.141592653589793238462643383279502884197;

  double tau1, tau2;    // motor torques
  int tau1_limit = 100; // limits on torques (motor contraints)
  int tau2_limit = 60;
  int kp = 100, kd = 10; // controller gains

  // Parts of equations of motion
  double det_M, w1_C, w2_C, w1_G, w2_G, w1_D, w2_D, term1, term2;
  double x_prev; // for saving x coord of previous frame

  std::array<int, 2> rotation_reference = {
      0, 0}; // to count the turns of the desired position
  bool controllerState = 1;
};

#endif
