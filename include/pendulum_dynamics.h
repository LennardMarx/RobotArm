#pragma once
#include <array>
#include <vector>

class PendulumDynamics
{
public:
    PendulumDynamics();		  // contructor
    virtual ~PendulumDynamics(); // destructor

    // get and set methods
    bool getReset();
    void setReset(bool);

    std::array<double, 2> getReceivedInputs();
    void setReceivedInputs(std::array<double, 2>);

    std::array<double, 4> getReceivedStates();
    void setReceivedStates(const std::array<double, 4>&);

    std::array<double, 4> getUpdatedStates();
    void setUpdatedStates(std::array<double, 4>);

    // virtual runge kutta methods to be overridden
    void rungeKutta();
    virtual std::array<double, 4> f(std::array<double, 4>, std::array<double, 2>&);
    std::array<double, 4> addArrays(std::array<double, 4>, std::array<double, 4>, double);

protected:			 // maybe change to private -> pass by reference
    double h = 0.001; // Time step

    bool reset = false;
    std::array<double, 4> receivedStates; // array to save received drone states
    std::array<double, 2> receivedInputs; // array to save reveiced inputs
    std::array<double, 4> updatedStates;  // calculated new drone states
    std::array<double, 4> K1, K2, K3, K4; // RK4 intermediate steps

    const double g = 9.81;	    // Gravity
    const double beta1 = 0.02; // Damping on joint 1
    const double beta2 = 0.03; // Damping on joint 2
    const float m1 = 0.45;		// Mass Link 1
    const float m2 = 0.15;		// Mass Link 2
    const float l1 = 0.2;		// Length Link 1
    const float l2 = 0.1;		// Length Link 2
};