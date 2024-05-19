#pragma once
#include <Eigen/Dense>

/* Observer class
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
*/
class FullStateObserver : public Observer{
private:
    int state_dim;
public:
    FullStateObserver(const int& state_dim_);
    void set_x_hat(const Eigen::VectorXd& x_hat_) override;
    void update(const Eigen::VectorXd& y, const Eigen::VectorXd& u, const double& dt) override;
};