#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "../include/pendulum.h"

using namespace std::chrono_literals;

Pendulum::Pendulum(double _th1, double _th2) // constructor
{
    setStates({ _th1, _th2, 0, 0 });
}
Pendulum::~Pendulum() {} // destructor

std::array<double, 4> Pendulum::getStates() // get function for the drone states
{
    return pendulumStates;
}
void Pendulum::setStates(std::array<double, 4> _x) // set function to the drone states
{
    pendulumStates = _x;
}

bool Pendulum::getReset() // get and set functions for the reset
{
    return reset;
}
void Pendulum::setReset(bool _reset)
{
    reset = _reset;
}