#define BOOST_TEST_MODULE test_system
#include <boost/test/included/unit_test.hpp>
#include "system.h"

BOOST_AUTO_TEST_CASE(correct_xdot){
    // Define ABCD matrices
    Eigen::MatrixXd A(2, 2);
    A << 1, 2, 3, 4;
    Eigen::MatrixXd B(2, 1);
    B << 5, 6;
    Eigen::MatrixXd C(1, 2);
    C << 7, 8;
    Eigen::MatrixXd D(1, 1);
    D << 9;
    // Define initial state vector
    Eigen::VectorXd x(2); x << 1, 2;
    // Define test input
    Eigen::VectorXd u(1);
    u << 7;

    // Test LinearSystem class
    LinearSystem linear_system(A, B, C, D);
    linear_system.set_x(x);
    Eigen::VectorXd xdot = linear_system.xdot(u);
    Eigen::VectorXd expected_xdot(2);
    expected_xdot << 40, 53;
    BOOST_CHECK(xdot.isApprox(expected_xdot));
}

BOOST_AUTO_TEST_CASE(correct_get_y){
    // Define ABCD matrices
    Eigen::MatrixXd A(2, 2);
    A << 1, 2, 3, 4;
    Eigen::MatrixXd B(2, 1);
    B << 5, 6;
    Eigen::MatrixXd C(1, 2);
    C << 7, 8;
    Eigen::MatrixXd D(1, 1);
    D << 9;
    // Define initial state vector
    Eigen::VectorXd x(2); x << 1, 2;
    // Define test input
    Eigen::VectorXd u(1);
    u << 7;

    // Test LinearSystem class
    LinearSystem linear_system(A, B, C, D);
    linear_system.set_x(x);
    Eigen::VectorXd y = linear_system.get_y(u);
    Eigen::VectorXd expected_y(1); expected_y << 86;
    BOOST_CHECK(y.isApprox(expected_y));
}

BOOST_AUTO_TEST_CASE(correct_update){
    // Define ABCD matrices
    Eigen::MatrixXd A(2, 2);
    A << 1, 2, 3, 4;
    Eigen::MatrixXd B(2, 1);
    B << 5, 6;
    Eigen::MatrixXd C(1, 2);
    C << 7, 8;
    Eigen::MatrixXd D(1, 1);
    D << 9;
    // Define initial state vector
    Eigen::VectorXd x(2); x << 1, 2;
    // Define test input
    Eigen::VectorXd u(1);
    u << 7;
    // Define time step
    double dt = 0.1;

    // Test LinearSystem class
    LinearSystem linear_system(A, B, C, D);
    linear_system.set_x(x);
    linear_system.update(u, dt);
    Eigen::VectorXd expected_x(2); expected_x << 5.0, 7.3;
    BOOST_CHECK(linear_system.get_x().isApprox(expected_x));
}

BOOST_AUTO_TEST_CASE(correct_spring_mass_xdot){
    // Define spring-mass system
    double m = 1.0;
    double k = 2.0;
    double b = 3.0;
    SpringMassSystem spring_mass_system(m, k, b);
    Eigen::VectorXd x(2); x << 1, 2;
    spring_mass_system.set_x(x);
    Eigen::VectorXd u(1); u << 7;
    Eigen::VectorXd xdot = spring_mass_system.xdot(u);
    Eigen::VectorXd expected_xdot(2); expected_xdot << 2, -1;
    BOOST_CHECK(xdot.isApprox(expected_xdot));
}