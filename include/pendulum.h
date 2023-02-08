#pragma once
#include <array>

// base drone class
class Pendulum
{
public:
    Pendulum(double, double); // constructor
    virtual ~Pendulum();	   // destructor

    // get and set methods
    std::array<double, 4> getStates();
    void setStates(std::array<double, 4>);

    bool getReset();
    void setReset(bool);

private:
    bool reset = false;
    std::array<double, 4> pendulumStates; // drone states
};
