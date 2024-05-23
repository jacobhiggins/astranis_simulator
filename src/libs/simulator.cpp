#include "simulator.h"
#include <iostream>
// #include <fstream>

Simulator::Simulator(std::unique_ptr<System> system_, 
        std::unique_ptr<Observer> observer_, 
        std::unique_ptr<Controller> controller_,
        const double& sim_dt_, const double& controller_dt_) : 
        system(std::move(system_)), observer(std::move(observer_)), controller(std::move(controller_)),
        t(0.0), sim_dt(sim_dt_), controller_dt(controller_dt_), simsteps_per_controllerstep(controller_dt/sim_dt) {
            if (sim_dt <= 0.0){throw std::invalid_argument("sim_dt must be greater than 0");}
            if (controller_dt <= 0.0){throw std::invalid_argument("controller_dt must be greater than 0");}
            if (controller_dt <= sim_dt){throw std::invalid_argument("controller_dt must be greater than or equal to sim_dt");}
        }

void Simulator::set_x(const Eigen::VectorXd& x_){system->set_x(x_); observer->set_x_hat(x_);}
void Simulator::set_xref(const Eigen::VectorXd& xref_){xref = xref_;}
void Simulator::set_print2console(const bool& print2console_){print2console = print2console_;}
void Simulator::set_print2file(const bool& print2file_){
    print2file = print2file_;
    if (print2file){
        output_file = std::ofstream(output_filepath);
        // output_file.open(output_filepath);
        if (!output_file.is_open()){
            throw std::invalid_argument("Could not open output file");
        }
    }
}
void Simulator::set_output_filepath(const std::string& output_filepath_){output_filepath = output_filepath_;}

void Simulator::sim_step() {
    Eigen::VectorXd x = observer->get_x_hat(); // Get current estimated state
    Eigen::VectorXd u = controller->control(x, xref); // Get control input
    for (int i=0; i<simsteps_per_controllerstep; ++i){
        apply_u(u); // Apply control input to the system
        if (print2console){
            std::cout << "\tt, x, u : " << t << " | " << system->get_x().transpose() << " | " << u.transpose() << ")" << std::endl;
        }
        if (print2file){
            output_file << t << "," << system->get_x()[0] << "," << system->get_x()[1] << "," << u[0] << ", " << xref[0] << std::endl;
        }
    }
}

void Simulator::apply_u(const Eigen::VectorXd& u) {
    observer->update(system->get_y(u), u, sim_dt); // Update observer state
    system->update(u, sim_dt); // Update system state
    t += sim_dt; // Update time
}

void Simulator::run(const double& duration){
    if (print2console) std::cout << "Running simulation for " << duration << " seconds" << std::endl;
    if (print2file) output_file << "t,pos,vel,u,pref" << std::endl;
    while (t < duration){
        sim_step();
    }
    if (print2file) output_file.close();
}