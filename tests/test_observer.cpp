#define BOOST_TEST_MODULE test_observer
#include <boost/test/included/unit_test.hpp>
#include "observer.h"
#include <iostream>

BOOST_AUTO_TEST_CASE(correct_get_x_hat){
    // Define FullStateObserver
    FullStateObserver full_state_observer(2);
    Eigen::VectorXd y(2); y << 1, 2; // Output of the system
    Eigen::VectorXd u(1); u << 1000; // Input of the system
    full_state_observer.update(y,u,0.1);
    Eigen::VectorXd expected_x_hat(2); expected_x_hat << 1, 2;
    BOOST_CHECK(full_state_observer.get_x_hat().isApprox(expected_x_hat));
}