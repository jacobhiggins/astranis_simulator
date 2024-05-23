#define BOOST_TEST_MODULE test_controller
#include <boost/test/included/unit_test.hpp>
#include "controller.h"
#include "system.h"
#include <math.h>
#include <limits>

BOOST_AUTO_TEST_CASE(correct_springmasspd_params){
    // Define FullStateObserver
    double m = 1.0;
    double k = 2.0;
    double b = 3.0;
    double decay_rate = 4.0;
    SpringMassSystem sys(m, k, b);
    SpringMassPDController controller(sys, decay_rate);
    double eps = std::numeric_limits<double>::epsilon();
    BOOST_CHECK(std::abs(controller.get_kp() - 14.0) < eps);
    BOOST_CHECK(std::abs(controller.get_kd() - 5.0) < eps);
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
    Eigen::VectorXd u_expected(1); u_expected << 24.0;
    BOOST_CHECK(u.isApprox(u_expected));
}