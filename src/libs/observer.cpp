#include <observer.h>

// Observer Methods
Eigen::VectorXd Observer::get_x_hat(){
    return x_hat;
}

void Observer::set_x_hat(const Eigen::VectorXd& x_hat_){
    x_hat = x_hat_;
}

// FullStateObserver Methods
FullStateObserver::FullStateObserver(const int& state_dim_){
    state_dim = state_dim_;
    set_x_hat(Eigen::VectorXd::Zero(state_dim));
}

void FullStateObserver::set_x_hat(const Eigen::VectorXd& x_hat_){
    if (x_hat_.size() != state_dim){
        throw std::invalid_argument("x_hat_ must have the same size as state_dim");
    }
    Observer::set_x_hat(x_hat_);
}

void FullStateObserver::update(const Eigen::VectorXd& y, const Eigen::VectorXd& u){
    if (y.size() != state_dim){
        throw std::invalid_argument("y must have the same size as state_dim for full-state observer");
    }
    set_x_hat(y);
}