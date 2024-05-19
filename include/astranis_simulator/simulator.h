#pragma once
#include <memory>
#include <string>
#include <fstream>
#include "system.h"
#include "observer.h"
#include "controller.h"

/* Simulator class
Simulates the closed-loop dynamical system with controller and observer
*/
class Simulator{
private:
    bool print2console = false;
    std::unique_ptr<System> system;
    std::unique_ptr<Observer> observer;
    std::unique_ptr<Controller> controller;
    Eigen::VectorXd xref;
    double t;
    double sim_dt;
    double controller_dt;
    // Output file
    bool print2file = false;
    std::string output_filepath = "./output.csv";
    std::ofstream output_file;
public:
    Simulator(std::unique_ptr<System> system_, 
            std::unique_ptr<Observer> observer_, 
            std::unique_ptr<Controller> controller_,
            const double& sim_dt_, const double& controller_dt_);
    void sim_step();
    void apply_u(const Eigen::VectorXd& u);
    void run(const double& duration);
    // Getters
    Eigen::VectorXd get_x();
    // Setters
    void set_x(const Eigen::VectorXd& x_);
    void set_xref(const Eigen::VectorXd& xref_);
    void set_print2console(const bool& print2console_);
    void set_print2file(const bool& print2file_);
    void set_output_filepath(const std::string& output_filepath_);
};