#pragma once
#include <array>
#include "../include/pendulum_dynamics.h"

// base drone class
class Pendulum
{
    friend class PendulumDynamics;
public:
    Pendulum(double, double); // constructor
    virtual ~Pendulum();	   // destructor

    // get and set methods
    std::array<double, 4> getStates();
    void setStates(std::array<double, 4>);

    bool getReset();
    void setReset(bool);

    std::array<const double, 2> getMass();
    // void setMass(std::array<double, 2>);

    std::array<double, 2> getLength();
    void setLength(std::array<double, 2>);

    std::array<double, 2> getDamping();
    void setDamping(std::array<double, 2>);

private:
    bool reset = false;
    std::array<double, 4> pendulumStates; // drone states

public:
    std::array<const double, 2> m = { 0.3, 0.3 }; // link masses
    static double m3;
    std::array<double, 2> l = { 0.5, 0.5 }; // link lengths
    std::array<double, 2> beta = { 0.02, 0.02 }; // joint damping
};
