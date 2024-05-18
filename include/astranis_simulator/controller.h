#pragma once
#include <Eigen/Dense>
#include "system.h"
/* Controller class
*/
class Controller{
public:
    virtual Eigen::VectorXd control(const Eigen::VectorXd& x, const Eigen::VectorXd& xref) = 0;
};

/* PID Controller class
*/
class SpringMassPDController : public Controller{
private:
    double m; // Mass
    double k; // Spring coefficient
    double b; // Damping coefficient
    double kp; // Proportional gain
    double kd; // Derivative gain
public:
    SpringMassPDController(const SpringMassSystem& sys, const double& kp_, const double& kd_);
    SpringMassPDController(const SpringMassSystem& sys, const double& decay_rate);
    Eigen::VectorXd control(const Eigen::VectorXd& x, const Eigen::VectorXd& xref) override;
    double get_kp();
    double get_kp() const;
    double get_kd();
    double get_kd() const;
};