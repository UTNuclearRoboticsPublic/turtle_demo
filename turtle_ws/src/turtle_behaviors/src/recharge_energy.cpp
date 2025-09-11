#include "turtle_behaviors/recharge_energy.hpp"

namespace turtle_behaviors {
NodeStatus RechargeEnergy::onStart() {
    double max_energy;
    if (!getInput("max_energy", max_energy)) {
        std::cout << '[' << name() << "] " << "ERROR: No max_energy found" << std::endl;
        return NodeStatus::FAILURE;
    };

    double energy_per_second;
    if (!getInput("energy_per_second", energy_per_second)) {
        std::cout << '[' << name() << "] " << "ERROR: No energy_per_second found" << std::endl;
        return NodeStatus::FAILURE;
    };

    double energy;
    if (!getInput("energy", energy)) {
        std::cout << '[' << name() << "] " << "ERROR: No energy found." << std::endl;
        return NodeStatus::FAILURE;
    };

    last_time_ = std::chrono::system_clock::now();

    std_msgs::msg::Float64 energy_msg;
    energy_msg.data = energy;
    setOutput("energy_msg", energy_msg);

    if (energy == max_energy) {
        std::cout << '[' << name() << "] " << "No recharge needed." << std::endl;
        return NodeStatus::SUCCESS;
    }
    else {
        std::cout << '[' << name() << "] " << "Charging..." << std::endl;
        return NodeStatus::RUNNING;
    }
}

NodeStatus RechargeEnergy::onRunning() {
    double max_energy, energy_per_second, energy;
    getInput("max_energy", max_energy);
    getInput("energy_per_second", energy_per_second);
    getInput("energy", energy);
    
    std::chrono::duration<float, std::chrono::seconds::period> time_span =
        std::chrono::system_clock::now() - last_time_;
    double energy_gain = energy_per_second * time_span.count();
    double new_energy = std::min(energy + energy_gain, max_energy);
    // std::cout << '[' << name() << "] " << "New energy: " << new_energy << std::endl;

    std_msgs::msg::Float64 energy_msg;
    energy_msg.data = new_energy;

    last_time_ = std::chrono::system_clock::now();

    setOutput("energy", new_energy);
    setOutput("energy_msg", energy_msg);

    if (new_energy >= max_energy) {
        std::cout << '[' << name() << "] " << "Fully charged." << std::endl;
        return NodeStatus::SUCCESS;
    }
    else return NodeStatus::RUNNING;
}

void RechargeEnergy::onHalted() {
    return;
}
} // turtle_behaviors