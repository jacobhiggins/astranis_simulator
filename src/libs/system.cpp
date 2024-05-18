#include "system.h"

// System methods
System::System(const int& state_dim_) : state_dim(state_dim_) {
    x = Eigen::VectorXd::Zero(state_dim);
}

void System::update(const Eigen::VectorXd& u, const double& dt) {
    x += xdot(u) * dt;
}

// LinearSystem methods
LinearSystem::LinearSystem(const Eigen::MatrixXd& A_, const Eigen::MatrixXd& B_, const Eigen::MatrixXd& C_, const Eigen::MatrixXd& D_) : System(A_.rows()) {
    // Check if the matrices have the correct dimensions
    if (A_.rows() != A_.cols() || A_.rows() != B_.rows() || A_.rows() != C_.cols() || A_.rows() != D_.rows() || B_.cols() != D_.cols() || C_.rows() != D_.cols()) {
        throw std::invalid_argument("Invalid matrix dimensions");
    }
    A = A_;
    B = B_;
    C = C_;
    D = D_;
}

Eigen::VectorXd LinearSystem::xdot(const Eigen::VectorXd& u) {
    return A * x + B * u;
}