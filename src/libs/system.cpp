#include "system.h"

// System methods
System::System(const int& state_dim_) : state_dim(state_dim_) {
    x = Eigen::VectorXd::Zero(state_dim);
}

void System::set_state_dim(const int& state_dim_) {
    state_dim = state_dim_;
}

void System::set_x(const Eigen::VectorXd& x_) {
    if (x_.size() != state_dim) {
        throw std::invalid_argument("Invalid state vector size");
    }
    x = x_;
}

Eigen::VectorXd System::get_x() {
    return x;
}

void System::update(const Eigen::VectorXd& u, const double& dt) {
    x += xdot(u) * dt;
}

// LinearSystem methods
LinearSystem::LinearSystem(const Eigen::MatrixXd& A_, const Eigen::MatrixXd& B_, const Eigen::MatrixXd& C_, const Eigen::MatrixXd& D_) : System(A_.rows()) {
    set_ABCD(A_, B_, C_, D_);
}

LinearSystem::LinearSystem() : System(0) {}

void LinearSystem::set_ABCD(const Eigen::MatrixXd& A_, const Eigen::MatrixXd& B_, const Eigen::MatrixXd& C_, const Eigen::MatrixXd& D_) {
    // Check if the matrices have the correct dimensions
    if (A_.rows()!=A_.cols()){throw std::invalid_argument("A matrix must be square");}
    if (A_.rows()!=B_.rows()){throw std::invalid_argument("A and B matrices must have the same number of rows");}
    if (A_.rows()!=C_.cols()){throw std::invalid_argument("A rows must be the same as C columns");}
    if (C_.rows()!=D_.rows()){throw std::invalid_argument("C and D matrices must have the same number of rows");}
    A = A_;
    B = B_;
    C = C_;
    D = D_;
    set_state_dim(A.rows());
}

Eigen::VectorXd LinearSystem::xdot(const Eigen::VectorXd& u) {
    if (u.size() != B.cols()) {
        throw std::invalid_argument("Invalid input vector size");
    }
    return A * x + B * u;
}

Eigen::VectorXd LinearSystem::get_y(const Eigen::VectorXd& u) {
    return C * x + D * u;
}

// SpringMassSystem methods
SpringMassSystem::SpringMassSystem(const double& m_, const double& k_, const double& b_)
    : m(m_), k(k_), b(b_) {
    Eigen::MatrixXd A(2,2); A << 0, 1, -k/m, -b/m;
    Eigen::MatrixXd B(2,1); B << 0, 1/m;
    Eigen::MatrixXd C(2,2); C << 1, 0 , 0, 1;
    Eigen::MatrixXd D(2,1); D << 0, 0;
    set_ABCD(A, B, C, D);
}

double SpringMassSystem::get_m() {return m;}
double SpringMassSystem::get_m() const {return m;}
double SpringMassSystem::get_k() {return k;}
double SpringMassSystem::get_k() const {return k;}
double SpringMassSystem::get_b() {return b;}
double SpringMassSystem::get_b() const {return b;}

// Noisy SpringMassSystem methods
NoisySpringMassSystem::NoisySpringMassSystem(const double& m_, const double& k_, const double& b_)
    : SpringMassSystem(m_, k_, b_) {}

void NoisySpringMassSystem::set_noise_std(const Eigen::Vector2d& noise_std_) {noise_std = noise_std_;}

void NoisySpringMassSystem::update(const Eigen::VectorXd& u, const double& dt) {
    x += xdot(u) * dt;
    x += Eigen::Vector2d::Random().cwiseProduct(noise_std)*dt; // Process noise
}