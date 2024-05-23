#pragma once
#include <Eigen/Dense>

/* Observer class
    The observer class is a abstract class for all observers. It has a state estimate x_hat.
    Required methods:
        - update: updates the state estimate x_hat given the measurement y, input u, and time step dt
*/
class Observer{
private:
    Eigen::VectorXd x_hat;
public:
    Eigen::VectorXd get_x_hat();
    virtual void set_x_hat(const Eigen::VectorXd& x_hat_);
    virtual void update(const Eigen::VectorXd& y, const Eigen::VectorXd& u, const double& dt) = 0;
};

/* Full state observer class
    The full state observer is an observer that takes in the full state of the system as the measurement,
    so it can pass the state directly to the controller. It does not do anything interesting, but simply
    is provided as a concreate class that may be instantiated.
*/
class FullStateObserver : public Observer{
private:
    int state_dim; // State dimension, included to ensure that the measurement y has the correct size
public:
    FullStateObserver(const int& state_dim_);
    void set_x_hat(const Eigen::VectorXd& x_hat_) override;
    void update(const Eigen::VectorXd& y, const Eigen::VectorXd& u, const double& dt) override;
};