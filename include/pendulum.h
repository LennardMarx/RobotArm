#ifndef PENDULUM_H
#define PENDULUM_H

#pragma once
#include <array>
#include "../include/pendulum_dynamics.h"

class Pendulum
{
    friend class PendulumDynamics;
public:
    Pendulum(); // constructor
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
    // variables of the robot arm
    // static -> immutable?
    // inline -> would else have to be declared (?) in .ccp
    static inline const double beta1 = 0.02; // Damping on joint 1
    static inline const double beta2 = 0.02; // Damping on joint 2
    static inline const double m1 = 1;		// Mass Link 1
    static inline const double m2 = 1;		// Mass Link 2
    static inline const double l1 = 0.5;		// Length Link 1
    static inline const double l2 = 0.5;		// Length Link 2
};

#endif