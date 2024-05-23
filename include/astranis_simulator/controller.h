#pragma once
#include <Eigen/Dense>
#include "system.h"
/* Controller class
    Abstract class for controllers within simulator
    - Required functions:
        - control: Calculate control input based on current state and reference state
*/
class Controller{
public:
    virtual Eigen::VectorXd control(const Eigen::VectorXd& x, const Eigen::VectorXd& xref) = 0;
};

/* PID Controller class
    PID controller for a spring-mass system
    - Parameters:
        - m: Mass
        - k: Spring coefficient
        - b: Damping coefficient
        - kp: Proportional gain
        - kd: Derivative gain
    - Functions:
        - control: Calculate control input based on current state and reference state
        - get_kp: Get proportional gain
        - get_kd: Get derivative gain
*/
class SpringMassPDController : public Controller{
private:
    double m; // Mass
    double k; // Spring coefficient
    double b; // Damping coefficient
    double kp; // Proportional gain
    double kd; // Derivative gain
public:
    SpringMassPDController(const SpringMassSystem& sys, const double& kp_, const double& kd_); // Constructor with specified kp and kd
    SpringMassPDController(const SpringMassSystem& sys, const double& decay_rate); // Constructor with decay rate (assuming critial damping)
    Eigen::VectorXd control(const Eigen::VectorXd& x, const Eigen::VectorXd& xref) override;
    // Getters
    double get_kp();
    double get_kp() const;
    double get_kd();
    double get_kd() const;
};