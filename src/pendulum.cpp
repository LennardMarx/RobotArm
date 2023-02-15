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

std::array<double, 4> Pendulum::getStates() { return pendulumStates; }
void Pendulum::setStates(std::array<double, 4> _x) { pendulumStates = _x; }

std::array<const double, 2> Pendulum::getMass() { return m; }
// void Pendulum::setMass(std::array<const double, 2> _m) { m = _m; }

std::array<double, 2> Pendulum::getLength() { return l; }
void Pendulum::setLength(std::array<double, 2> _l) { l = _l; }

std::array<double, 2> Pendulum::getDamping() { return beta; }
void Pendulum::setDamping(std::array<double, 2> _beta) { beta = _beta; }

bool Pendulum::getReset() { return reset; }
void Pendulum::setReset(bool _reset) { reset = _reset; }