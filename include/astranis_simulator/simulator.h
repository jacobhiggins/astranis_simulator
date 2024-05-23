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
    bool print2console = false;                     // Print to console
    std::unique_ptr<System> system;                 // Dynamical system to be simulated
    std::unique_ptr<Observer> observer;             // Observer for state estimation
    std::unique_ptr<Controller> controller;         // Controller for closed-loop  control input
    Eigen::VectorXd xref;                           // Reference state for the controller
    double t;                                       // Current time in simulation
    double sim_dt;                                  // Simulation time step
    double controller_dt;                           // Controller time step
    int simsteps_per_controllerstep;                // Number of simulation steps per controller step
    bool print2file = false;                        // Print to file    
    std::string output_filepath = "./output.csv";   // File path for the output file
    std::ofstream output_file;                      // Output file stream
    void sim_step();                                // Perform a single simulation step
    void apply_u(const Eigen::VectorXd& u);         // Apply control input to the system
public:
    Simulator(std::unique_ptr<System> system_, 
            std::unique_ptr<Observer> observer_, 
            std::unique_ptr<Controller> controller_,
            const double& sim_dt_, const double& controller_dt_);
    void run(const double& duration);               // Rune the simulation for a given duration
    // Getters
    Eigen::VectorXd get_x();
    // Setters
    void set_x(const Eigen::VectorXd& x_);
    void set_xref(const Eigen::VectorXd& xref_);
    void set_print2console(const bool& print2console_);
    void set_print2file(const bool& print2file_);
    void set_output_filepath(const std::string& output_filepath_);
};