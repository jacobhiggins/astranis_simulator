#pragma once
#include <Eigen/Dense>

/* System class
Abstract class that represents a dynamical system
Required methods:
    - xdot: returns the derivative of the state vector x given the input vector u and the time step dt
*/
class System {
private:
    int state_dim;
protected:
    Eigen::VectorXd x;
public:
    System(const int& state_dim_);
    virtual Eigen::VectorXd xdot(const Eigen::VectorXd& u) = 0;
    void update(const Eigen::VectorXd& u, const double& dt);
};

/* Linear System class
Inherits from the System class
Represents a linear dynamical system defined by the matrices A, B, C, and D
*/
class LinearSystem : public System {
private:
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd C;
    Eigen::MatrixXd D;
public:
    LinearSystem(const Eigen::MatrixXd& A_, const Eigen::MatrixXd& B_, const Eigen::MatrixXd& C_, const Eigen::MatrixXd& D_);
    Eigen::VectorXd xdot(const Eigen::VectorXd& u) override;
};