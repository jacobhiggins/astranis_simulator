#include <iostream>
#include <memory>
#include "system.h"
#include "controller.h"
#include "observer.h"
#include "simulator.h"
#include <nlohmann/json.hpp>
#include <fstream>

int main(int argc, char** argv) {
    // ----------------- Load Config File -----------------
    if (argc < 2){
        std::cerr << "Usage: " << argv[0] << " <config_file.json> <output_file_path> (optional)" << std::endl;
        return 1;
    }
    std::ifstream config_file(argv[1]);
    nlohmann::json config; config_file >> config;

    // ----------------- System Params -----------------
    // Define the system parameters
    double m = config["system"]["mass"];                    // mass
    double k = config["system"]["spring_constant"];         // spring constant
    double b = config["system"]["damping_coefficient"];     // damping coefficient

    // ----------------- Controller Params -----------------
    double decay_rate = config["controller"]["decay_rate"]; // decay rate

    // ----------------- State Conditions -----------------
    Eigen::VectorXd x0(2);
    x0 << config["state_conditions"]["x0"][0], config["state_conditions"]["x0"][1];        // Initial state
    Eigen::VectorXd xref(2);
    xref << config["state_conditions"]["xref"][0], config["state_conditions"]["xref"][1];  // Reference state

    // ----------------- Simulator Params -----------------
    double tf = config["simulator"]["final_time"];               // final time
    double sim_dt = config["simulator"]["simulation_dt"];        // simulation time step
    double controller_dt = config["simulator"]["controller_dt"]; // controller time step

    // Create a SpringMassSystem object
    std::unique_ptr<SpringMassSystem> sys = std::make_unique<SpringMassSystem>(m, k, b);
    // Create a SpringMassPDController object
    std::unique_ptr<SpringMassPDController> controller = std::make_unique<SpringMassPDController>(*sys, decay_rate);
    // Create an Observer object
    std::unique_ptr<FullStateObserver> observer = std::make_unique<FullStateObserver>(2);
    // Create a Simulator object
    Simulator sim(std::move(sys), std::move(observer), std::move(controller), sim_dt, controller_dt);
    sim.set_x(x0);
    sim.set_xref(xref);
    sim.set_print2console(true);
    if (argc > 2) {sim.set_output_filepath(argv[2]); sim.set_print2file(true);}

    // ----------------- Run Simulation -----------------
    sim.run(tf);

    return 0;
}