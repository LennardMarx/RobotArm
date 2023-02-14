#pragma once
#include <array>
#include <vector>

class PendulumDynamics
{
    // friend class Pendulum;
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

    std::array<double, 2> inverseKinematics(std::array<double, 2>);

    void toggleController();

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
    const double beta2 = 0.02; // Damping on joint 2
    const double m1 = 0.3;		// Mass Link 1
    const double m2 = 0.3;		// Mass Link 2
    const double l1 = 0.5;		// Length Link 1
    const double l2 = 0.5;		// Length Link 2
    const double pi = 3.141592653589793238462643383279502884197;
    double tau1, tau2;
    // limits on torques
    int tau1_limit = 100;
    int tau2_limit = 60;
    int kp, kd;
    // Parts of equations of motion
    double det_M, w1_C, w2_C, w1_G, w2_G, w1_D, w2_D, term1, term2;
    double x_prev; // for saving x coord of previous frame

    int turncounter = 0; // to count the turns of the desired position
    bool controllerState = 1; // 1 = controller on, 0 = off
};