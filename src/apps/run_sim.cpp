#include <iostream>
#include "system.h"

int main() {
    std::cout << "Hello, Simulation!" << std::endl;

    double m = 1.0;
    double k = 1.0;
    double b = 0.1;

    // Define the system matrices
    Eigen::MatrixXd A(2, 2); A << 0, 1, 0, 1;

    return 0;
}