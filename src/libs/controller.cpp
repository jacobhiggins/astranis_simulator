#include "controller.h"

SpringMassPDController::SpringMassPDController(const SpringMassSystem& sys, const double& kp_, const double& kd_) : kp(kp_), kd(kd_) {
    m = sys.get_m();
    k = sys.get_k();
    b = sys.get_b();
}

SpringMassPDController::SpringMassPDController(const SpringMassSystem& sys, const double& decay_rate) {
    m = sys.get_m();
    k = sys.get_k();
    b = sys.get_b();
    kp = m * decay_rate * decay_rate - k;
    kd = 2.0 * m * decay_rate - b;
}

Eigen::VectorXd SpringMassPDController::control(const Eigen::VectorXd& x, const Eigen::VectorXd& xref) {
    double pref = xref[0]; // reference position to be tracked
    double p = x[0];    // current position
    double v = x[1];    // current velocity
    Eigen::VectorXd u(1);
    u << (k * pref + kp * (pref - p) - kd * v) / m;
    return u;
}

double SpringMassPDController::get_kp() {return kp;}
double SpringMassPDController::get_kp() const {return kp;}
double SpringMassPDController::get_kd() {return kd;}
double SpringMassPDController::get_kd() const {return kd;}