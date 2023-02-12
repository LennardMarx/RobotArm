#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include "../include/pendulum_dynamics.h"
#include <vector>

PendulumDynamics::PendulumDynamics() // constructor
{
    // initial states
    setUpdatedStates({ 0, 0, 0, 0 });
    setReceivedStates({ 0, 0, 0, 0 });
    setReceivedInputs({ 0, 0 });
}
PendulumDynamics::~PendulumDynamics() {} // destructor

// get and set functions
bool PendulumDynamics::getReset()
{
    return reset;
}
void PendulumDynamics::setReset(bool _setup)
{
    reset = _setup;
}

std::array<double, 2> PendulumDynamics::getReceivedInputs()
{
    return receivedInputs;
}
void PendulumDynamics::setReceivedInputs(std::array<double, 2> _u)
{
    receivedInputs = _u;
}

std::array<double, 4> PendulumDynamics::getReceivedStates()
{
    return receivedStates;
}
void PendulumDynamics::setReceivedStates(const std::array<double, 4>& _states)
{
    receivedStates = _states;
}

std::array<double, 4> PendulumDynamics::getUpdatedStates()
{
    return updatedStates;
}
void PendulumDynamics::setUpdatedStates(std::array<double, 4> _xdot)
{
    updatedStates = _xdot;
}

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
        _updatedStates.at(i) = receivedStates.at(i) + (h * ((K1.at(i) + 2 * K2.at(i) + 2 * K3.at(i) + K4.at(i)) / 6));
    }
    setUpdatedStates(_updatedStates);
}

// the method to integrate the runge kutta intermediate steps

// gerenric equations of motion (absolute angles, no damping)
// std::array<double, 4> PendulumDynamics::f(std::array<double, 4> _states, std::array<double, 2>& _u)
// {
//     std::array<double, 4> _output;
//     _output.at(0) = _states.at(2);
//     _output.at(1) = _states.at(3);
//     double denom;
//     denom = l1 * (2 * m1 + m2 - m2 * cos(2 * _states.at(0) - 2 * _states.at(1)));
//     _output.at(2) = (-g * (2 * m1 + m2) * sin(_states.at(0)) - m2 * g * sin(_states.at(0) - 2 * _states.at(1)) - 2 * sin(_states.at(0) - _states.at(1)) * m2 * (pow(_states.at(3), 2) * l2 + pow(_states.at(2), 2) * l1 * cos(_states.at(0) - _states.at(1)))) / denom;
//     _output.at(3) = 2 * sin(_states.at(0) - _states.at(1)) * (pow(_states.at(2), 2) * l1 * (m1 + m2) + g * (m1 + m2) * cos(_states.at(0)) + pow(_states.at(3), 2) * l2 * m2 * cos(_states.at(0) - _states.at(1))) / denom;
//     return _output;
// }

// own dynamics of motion (matrix form, relative angles, damping)
std::array<double, 4> PendulumDynamics::f(std::array<double, 4> _states, std::array<double, 2>& _u)
{
    std::array<double, 4> _output;
    // theta1_dot
    _output.at(0) = _states.at(2);
    // theta2_dot
    _output.at(1) = _states.at(3);

    // Inertia determinant
    double det_M = (((m1 + m2) * l1 * l1 + m2 * l2 * l2 + 2 * m2 * l1 * l2 * cos(_states.at(1))) * m2 * l2 * l2) - ((m2 * l2 * l2 + m2 * l1 * l2 * cos(_states.at(1))) * (m2 * l2 * l2 + m2 * l1 * l2 * cos(_states.at(1))));

    // Coriolis forces
    double w1_C = (-m2 * l1 * l2 * (2 * _states.at(2) + _states.at(3)) * sin(_states.at(1))) * _states.at(3);
    double w2_C = (m2 * l1 * l2 * _states.at(2) * sin(_states.at(1))) * _states.at(2);

    // Gravity terms
    double w1_G = ((m1 + m2) * l1 * sin(_states.at(0)) + m2 * l2 * sin(_states.at(0) + _states.at(1))) * g;
    double w2_G = (m2 * l2 * sin(_states.at(0) + _states.at(1))) * g;

    // Damping
    double w1_D = beta1 * _states.at(2);
    double w2_D = beta2 * _states.at(3);

    // Control Law / Input
    double tau1 = 50 * (_u.at(0) - _states.at(0)) - 5 * (_states.at(2)) + ((m1 + m2) * l1 * sin(_u.at(0)) + m2 * l2 * sin(_u.at(0) + _u.at(1))) * g;
    double tau2 = 50 * (_u.at(1) - _states.at(1)) - 5 * (_states.at(3)) + (m2 * l2 * sin(_u.at(0) + _u.at(1))) * g;
    // tau1 = 0;
    // tau2 = 0;
    if (controllerOff)
    {
        tau1 = 0;
        tau2 = 0;
    }

    // terms to be multiplied with M_inv matrix
    double term1 = tau1 - w1_C - w1_G - w1_D;
    double term2 = tau2 - w2_C - w2_G - w2_D;


    // w1_dot
    _output.at(2) = ((m2 * l2 * l2) / det_M * term1 + (-m2 * l2 * l2 - m2 * l1 * l2 * cos(_states.at(1))) / det_M * term2);
    // w2_dot
    _output.at(3) = (-m2 * l2 * l2 - m2 * l1 * l2 * cos(_states.at(1))) / det_M * term1 + ((m1 + m2) * l1 * l1 + m2 * l2 * l2 + 2 * m2 * l1 * l2 * cos(_states.at(1))) / det_M * term2;

    // std::cout << tau1 << std::endl;

    return _output;
}

std::array<double, 2> PendulumDynamics::inverseKinematics(std::array<double, 2> _pos)
{
    std::array<double, 2> angles;
    double x = _pos.at(0);
    double y = _pos.at(1);
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

    // if (y < 0 && y_prev > 0 && x < 0)
    // {
    //     turncounter -= 1;
    // }
    // calculating inverse kinematics always in quadtrant 1 and transforming to other quadrants!
    if (x >= 0 && y >= 0)
    {
        angles.at(1) = acos((x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2));
        angles.at(0) = atan(y / x) - atan((l2 * sin(angles.at(1))) / (l1 + l2 * cos(angles.at(1)))) + pi / 2;
    }
    else if (x < 0 && y < 0)
    {
        angles.at(1) = acos((x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2));
        angles.at(0) = pi + atan(y / x) - atan((l2 * sin(angles.at(1))) / (l1 + l2 * cos(angles.at(1)))) + pi / 2;
    }
    else if (x >= 0 && y < 0)
    {
        x0 = -y; y0 = x; y = y0; x = x0; // flipping coordinates for quad. 4
        angles.at(1) = acos((x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2));
        angles.at(0) = -pi / 2 + atan(y / x) - atan((l2 * sin(angles.at(1))) / (l1 + l2 * cos(angles.at(1)))) + pi / 2;
    }
    else if (x < 0 && y >= 0)
    {
        x0 = y; y0 = -x; y = y0; x = x0; // flipping coordinates for quad. 2
        angles.at(1) = acos((x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2));
        angles.at(0) = pi / 2 + atan(y / x) - atan((l2 * sin(angles.at(1))) / (l1 + l2 * cos(angles.at(1)))) + pi / 2;
    }
    // std::cout << angles.at(0) * 180 / pi << ", " << angles.at(1) * 180 / pi << std::endl;

    angles.at(0) += turncounter * 2 * pi;

    return angles;
}

void PendulumDynamics::switchControllerState()
{
    controllerOff = !controllerOff;
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
