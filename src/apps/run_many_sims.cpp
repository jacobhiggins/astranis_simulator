#include <iostream>
#include <memory>
#include "system.h"
#include "controller.h"
#include "observer.h"
#include "simulator.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <random>
#include <vector>

int main(int argc, char** argv) {
    // ----------------- Load Config File -----------------
    if (argc < 2){
        std::cerr << "Usage: " << argv[0] << " <config_file.json> <output_file_path> (optional)" << std::endl;
        return 1;
    }
    std::ifstream config_file(argv[1]);
    nlohmann::json config; config_file >> config;

    // ----------------- Many Sim Params -----------------
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0,1.0);


    int num_sims = config["num_sims"];
    double tf = config["simulator"]["final_time"];               // final time
    double sim_dt = config["simulator"]["simulation_dt"];        // simulation time step
    double controller_dt = config["simulator"]["controller_dt"]; // controller time step
    Eigen::VectorXd xref(2), noise_std(2);
    xref << config["state_conditions"]["xref"][0], config["state_conditions"]["xref"][1];  // Reference state
    noise_std << config["system"]["noise_std"][0], config["system"]["noise_std"][1];  // Noise standard deviation
    std::vector<double> m_range, k_range, b_range, decay_rate_range, p0_range, v0_range;
    
    if (config["system"]["mass_range"].is_array()){
        for (int i = 0; i < 2; ++i) m_range.push_back(config["system"]["mass_range"][i]);
    } else {
        m_range.push_back(config["system"]["mass_range"]);
    }
    if (config["system"]["spring_constant_range"].is_array()){
        for (int i = 0; i < 2; ++i) k_range.push_back(config["system"]["spring_constant_range"][i]);
    } else {
        k_range.push_back(config["system"]["spring_constant_range"]);
    }
    if (config["system"]["damping_coefficient_range"].is_array()){
        for (int i = 0; i < 2; ++i) b_range.push_back(config["system"]["damping_coefficient_range"][i]);
    } else {
        b_range.push_back(config["system"]["damping_coefficient_range"]);
    }
    std::cout << "***************" << std::endl;
    if (config["controller"]["decay_rate_range"].is_array()){
        for (int i = 0; i < 2; ++i) decay_rate_range.push_back(config["controller"]["decay_rate_range"][i]);
    } else {
        decay_rate_range.push_back(config["controller"]["decay_rate_range"]);
    }
    if (config["state_conditions"]["p0_range"].is_array()){
        for (int i = 0; i < 2; ++i) p0_range.push_back(config["state_conditions"]["p0_range"][i]);
    } else {
        p0_range.push_back(config["state_conditions"]["p0_range"]);
    }
    if (config["state_conditions"]["v0_range"].is_array()){
        for (int i = 0; i < 2; ++i) v0_range.push_back(config["state_conditions"]["v0_range"][i]);
    } else {
        v0_range.push_back(config["state_conditions"]["v0_range"]);
    }

    // ----------------- Run Many Simulations -----------------
    
    for (int i=0; i<num_sims; i++){
        std::cout << "Running simulation " << i+1 << " of " << num_sims << std::endl;

        // ----------------- System Params -----------------
        // Define the system parameters
        double m = (m_range.size()==1) ? m_range[0] : (m_range[1]-m_range[0])*dist(gen) + m_range[0];
        double k = (k_range.size()==1) ? k_range[0] : (k_range[1]-k_range[0])*dist(gen) + k_range[0];
        double b = (b_range.size()==1) ? b_range[0] : (b_range[1]-b_range[0])*dist(gen) + b_range[0];

        // ----------------- Controller Params -----------------
        double decay_rate = (decay_rate_range.size()==1) ? decay_rate_range[0] : (decay_rate_range[1]-decay_rate_range[0])*dist(gen) + decay_rate_range[0];

        // ----------------- State Conditions -----------------
        Eigen::VectorXd x0(2);
        double p0 = (p0_range.size()==1) ? p0_range[0] : (p0_range[1]-p0_range[0])*dist(gen) + p0_range[0];
        double v0 = (v0_range.size()==1) ? v0_range[0] : (v0_range[1]-v0_range[0])*dist(gen) + v0_range[0];
        x0 << p0, v0;        // Initial state
        
        // Create a SpringMassSystem object
        // --> Noiseless 
        // std::unique_ptr<SpringMassSystem> sys = std::make_unique<SpringMassSystem>(m, k, b);
        // ---> Noisy
        std::unique_ptr<NoisySpringMassSystem> sys = std::make_unique<NoisySpringMassSystem>(m, k, b);
        sys->set_noise_std(noise_std);
        // Create a SpringMassPDController object
        std::unique_ptr<SpringMassPDController> controller = std::make_unique<SpringMassPDController>(*sys, decay_rate);
        // Create an Observer object
        std::unique_ptr<FullStateObserver> observer = std::make_unique<FullStateObserver>(2);
        // Create a Simulator object
        Simulator sim(std::move(sys), std::move(observer), std::move(controller), sim_dt, controller_dt);
        sim.set_x(x0);
        sim.set_xref(xref);
        // sim.set_print2console(true);
        if (argc > 2) {sim.set_output_filepath(std::string(argv[2])+std::to_string(i+1)+".csv"); sim.set_print2file(true);}

        // ----------------- Run Simulation -----------------
        sim.run(tf);
    }

    return 0;
}