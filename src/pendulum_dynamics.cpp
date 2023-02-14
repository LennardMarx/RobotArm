#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include "../include/pendulum_dynamics.h"
#include <vector>
#include "../include/pendulum.h"

PendulumDynamics::PendulumDynamics() // constructor
{
    // initial states
    setUpdatedStates({ 0, 0, 0, 0 });
    setReceivedStates({ 0, 0, 0, 0 });
    setReceivedInputs({ 0, 0 });
}
PendulumDynamics::~PendulumDynamics() {} // destructor

// get and set functions
bool PendulumDynamics::getReset() { return reset; }

void PendulumDynamics::setReset(bool _setup) { reset = _setup; }

std::array<double, 2> PendulumDynamics::getReceivedInputs() { return receivedInputs; }
void PendulumDynamics::setReceivedInputs(std::array<double, 2> _u) { receivedInputs = _u; }

std::array<double, 4> PendulumDynamics::getReceivedStates() { return receivedStates; }
void PendulumDynamics::setReceivedStates(const std::array<double, 4>& _states) { receivedStates = _states; }

std::array<double, 4> PendulumDynamics::getUpdatedStates() { return updatedStates; }
void PendulumDynamics::setUpdatedStates(std::array<double, 4> _xdot) { updatedStates = _xdot; }

// runge kutta method calculating the intermediate steps
void PendulumDynamics::rungeKutta()
{
    std::array<double, 4> _updatedStates;
    K1 = f(receivedStates, receivedInputs);
    K2 = f(addArrays(receivedStates, K1, 2), receivedInputs);
    K3 = f(addArrays(receivedStates, K2, 2), receivedInputs);
    K4 = f(addArrays(receivedStates, K3, 1), receivedInputs);
    for (int i = 0; i < 4; ++i)
    {
        _updatedStates[i] = receivedStates[i] + (h * ((K1[i] + 2 * K2[i] + 2 * K3[i] + K4[i]) / 6));
    }
    setUpdatedStates(_updatedStates);
}

// the method to integrate the runge kutta intermediate steps
// dynamics of motion (matrix form, relative _angles, damping)
std::array<double, 4> PendulumDynamics::f(std::array<double, 4> _states, std::array<double, 2>& _u)
{
    std::array<double, 4> _output;
    // theta1_dot
    _output[0] = _states[2];
    // theta2_dot
    _output[1] = _states[3];

    // Inertia determinant
    det_M = (((m1 + m2) * l1 * l1 + m2 * l2 * l2 + 2 * m2 * l1 * l2 * cos(_states[1])) * m2 * l2 * l2) - ((m2 * l2 * l2 + m2 * l1 * l2 * cos(_states[1])) * (m2 * l2 * l2 + m2 * l1 * l2 * cos(_states[1])));

    // Coriolis forces
    w1_C = (-m2 * l1 * l2 * (2 * _states[2] + _states[3]) * sin(_states[1])) * _states[3];
    w2_C = (m2 * l1 * l2 * _states[2] * sin(_states[1])) * _states[2];

    // Gravity terms
    w1_G = ((m1 + m2) * l1 * sin(_states[0]) + m2 * l2 * sin(_states[0] + _states[1])) * g;
    w2_G = (m2 * l2 * sin(_states[0] + _states[1])) * g;

    // Damping
    w1_D = beta1 * _states[2];
    w2_D = beta2 * _states[3];

    if (controllerState)
    {
        // Control Law / Input, kp, kd, gravity compensation
        kp = 100; kd = 10;
        tau1 = kp * (_u[0] - _states[0]) - kd * (_states[2]) + ((m1 + m2) * l1 * sin(_u[0]) + m2 * l2 * sin(_u[0] + _u[1])) * g;
        tau2 = kp * (_u[1] - _states[1]) - kd * (_states[3]) + (m2 * l2 * sin(_u[0] + _u[1])) * g;

        // define max motor strength
        if (tau1 > tau1_limit) { tau1 = tau1_limit; }
        if (tau1 < -tau1_limit) { tau1 = -tau1_limit; }
        if (tau2 > tau2_limit) { tau2 = tau2_limit; }
        if (tau2 < -tau2_limit) { tau2 = -tau2_limit; }
    }
    // when controller is switched off
    else if (!controllerState)
    {
        tau1 = 0;
        tau2 = 0;
    }
    // terms to be multiplied with M_inv matrix
    term1 = tau1 - w1_C - w1_G - w1_D;
    term2 = tau2 - w2_C - w2_G - w2_D;

    // anglular accelerations omega1_dot and omega2_dot
    _output[2] = ((m2 * l2 * l2) / det_M * term1 + (-m2 * l2 * l2 - m2 * l1 * l2 * cos(_states[1])) / det_M * term2);
    _output[3] = (-m2 * l2 * l2 - m2 * l1 * l2 * cos(_states[1])) / det_M * term1 + ((m1 + m2) * l1 * l1 + m2 * l2 * l2 + 2 * m2 * l1 * l2 * cos(_states[1])) / det_M * term2;

    return _output;
}

std::array<double, 2> PendulumDynamics::inverseKinematics(std::array<double, 2> _pos)
{
    std::array<double, 2> _angles;
    double x = _pos[0];
    double y = _pos[1];
    double x0, y0;

    // counting turn if endeffector crosses origin (+- 2 pi)
    if (x < 0 && x_prev >= 0 && y < 0)
    {
        turncounter -= 1;
    }
    else if (x >= 0 && x_prev < 0 && y < 0)
    {
        turncounter += 1;
    }
    //saving previous x position
    x_prev = x;
    //std::cout << x << ", " << y << "," << turncounter << std::endl;

    // calculating inverse kinematics always in quadtrant 1 and transforming to other quadrants!
    if (x >= 0 && y >= 0)
    {
        _angles[1] = acos((x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2));
        _angles[0] = atan(y / x) - atan((l2 * sin(_angles.at(1))) / (l1 + l2 * cos(_angles.at(1)))) + pi / 2;
    }
    else if (x < 0 && y < 0)
    {
        _angles[1] = acos((x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2));
        _angles[0] = pi + atan(y / x) - atan((l2 * sin(_angles.at(1))) / (l1 + l2 * cos(_angles.at(1)))) + pi / 2;
    }
    else if (x >= 0 && y < 0)
    {
        x0 = -y; y0 = x; y = y0; x = x0; // flipping coordinates for quad. 4
        _angles[1] = acos((x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2));
        _angles[0] = -pi / 2 + atan(y / x) - atan((l2 * sin(_angles.at(1))) / (l1 + l2 * cos(_angles.at(1)))) + pi / 2;
    }
    else if (x < 0 && y >= 0)
    {
        x0 = y; y0 = -x; y = y0; x = x0; // flipping coordinates for quad. 2
        _angles[1] = acos((x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2));
        _angles[0] = pi / 2 + atan(y / x) - atan((l2 * sin(_angles.at(1))) / (l1 + l2 * cos(_angles.at(1)))) + pi / 2;
    }
    // std::cout << _angles.at(0) * 180 / pi << ", " << _angles.at(1) * 180 / pi << std::endl;

    _angles[0] += turncounter * 2 * pi;

    return _angles;
}

void PendulumDynamics::toggleController()
{
    controllerState = !controllerState;
}

// method to add each position of an array with the according position of another
// _over to account for the multiplication with h/2 or h in the runge kutta steps
std::array<double, 4> PendulumDynamics::addArrays(std::array<double, 4> _x, std::array<double, 4> _K, double _over)
{
    std::array<double, 4> addedArray;
    for (int i = 0; i < 4; ++i)
    {
        addedArray.at(i) = _x.at(i) + _K.at(i) * h / _over;
    }
    return addedArray;
}
