#pragma once
#include <Eigen/Dense>

/* System class
Abstract class that represents a dynamical system
Required methods:
    - xdot: returns the derivative of the state vector x given the input vector u and the time step dt
    - get_y: returns the output vector y given the input vector u and (internally) the state vector x
*/
class System {
private:
    int state_dim;
protected:
    Eigen::VectorXd x;
    void set_state_dim(const int& state_dim_);
public: 
    System(const int& state_dim_);
    Eigen::VectorXd get_x();
    void set_x(const Eigen::VectorXd& x_);
    virtual Eigen::VectorXd xdot(const Eigen::VectorXd& u) = 0;
    virtual Eigen::VectorXd get_y(const Eigen::VectorXd& u) = 0;
    virtual void update(const Eigen::VectorXd& u, const double& dt);
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
protected:
    LinearSystem();
    void set_ABCD(const Eigen::MatrixXd& A_, const Eigen::MatrixXd& B_, const Eigen::MatrixXd& C_, const Eigen::MatrixXd& D_);
public:
    LinearSystem(const Eigen::MatrixXd& A_, const Eigen::MatrixXd& B_, const Eigen::MatrixXd& C_, const Eigen::MatrixXd& D_);
    Eigen::VectorXd xdot(const Eigen::VectorXd& u) override;
    Eigen::VectorXd get_y(const Eigen::VectorXd& u) override;
};

/* Spring-mass system class

*/
class SpringMassSystem : public LinearSystem {
private:
    double m; // Mass
    double k; // Spring coefficient
    double b; // Damping coefficient
public:
    SpringMassSystem(const double& m_, const double& k_, const double& b_);
    double get_m();
    double get_m() const;
    double get_k();
    double get_k() const;
    double get_b();
    double get_b() const;
};

/* Noisy Spring-mass system class

*/
class NoisySpringMassSystem : public SpringMassSystem {
private:
    Eigen::Vector2d noise_std = Eigen::Vector2d::Zero();
public:
    NoisySpringMassSystem(const double& m_, const double& k_, const double& b_);
    void update(const Eigen::VectorXd& u, const double& dt) override;
    void set_noise_std(const Eigen::Vector2d& noise_std_);
};