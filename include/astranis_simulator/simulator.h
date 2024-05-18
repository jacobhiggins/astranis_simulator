#pragma once
#include "system.h"
#include "observer.h"
#include "controller.h"

/* Simulator class
Simulates the closed-loop dynamical system with controller and observer
*/
class Simulator{
private:
    System* system;
    Observer* observer;
    Controller* controller;
    Eigen::VectorXd xref;
    double t;
    double sim_dt;
    double controller_dt;
public:
    Simulator(System* system_, Observer* observer_, Controller* controller_, const double& sim_dt_, const double& controller_dt_);
    void sim_step();
    void run(const double& duration);
    // Getters
    Eigen::VectorXd get_x();
    // Setters
    void set_x(const Eigen::VectorXd& x_);
    void set_xref(const Eigen::VectorXd& xref_);
};