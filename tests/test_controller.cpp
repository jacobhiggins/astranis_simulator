#define BOOST_TEST_MODULE test_controller
#include <boost/test/included/unit_test.hpp>
#include "controller.h"
#include "system.h"
#include "common.h"
#include <math.h>

BOOST_AUTO_TEST_CASE(correct_springmasspd_params){
    // Define FullStateObserver
    double m = 1.0;
    double k = 2.0;
    double b = 3.0;
    double decay_rate = 4.0;
    SpringMassSystem sys(m, k, b);
    SpringMassPDController controller(sys, decay_rate);
    BOOST_CHECK(std::abs(controller.get_kp() - 14.0) < EPS);
    BOOST_CHECK(std::abs(controller.get_kd() - 5.0) < EPS);
}

BOOST_AUTO_TEST_CASE(correct_springmasspd_control){
    // Define FullStateObserver
    double m = 1.0;
    double k = 2.0;
    double b = 3.0;
    double decay_rate = 4.0;
    SpringMassSystem sys(m, k, b);
    SpringMassPDController controller(sys, decay_rate);
    Eigen::VectorXd x(2); x << 1, 2;
    Eigen::VectorXd xref(2); xref << 3, 0;
    Eigen::VectorXd u = controller.control(x, xref);
    BOOST_CHECK(std::abs(u[0] - 24.0) < EPS);
}