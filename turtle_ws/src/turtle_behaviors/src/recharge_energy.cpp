#include "turtle_behaviors/recharge_energy.hpp"

namespace turtle_behaviors {
NodeStatus RechargeEnergy::onStart() {
    std::cout << '[' << name() << "] " << "Charging..." << std::endl;

    double max_energy;
    if (!getInput("max_energy", max_energy)) {
        std::cout << '[' << name() << "] " << "WARNING: No max_energy found" << std::endl;
        return NodeStatus::FAILURE;
    };

    double energy_per_tick;
    if (!getInput("energy_per_tick", energy_per_tick)) {
        std::cout << '[' << name() << "] " << "WARNING: No energy_per_tick found" << std::endl;
        return NodeStatus::FAILURE;
    };

    double energy;
    if (!getInput("energy", energy)) {
        std::cout << '[' << name() << "] " << "ERROR: No energy found." << std::endl;
        return NodeStatus::FAILURE;
    };

    return NodeStatus::RUNNING;
}

NodeStatus RechargeEnergy::onRunning() {
    double max_energy, energy_per_tick, energy;
    getInput("max_energy", max_energy);
    getInput("energy_per_tick", energy_per_tick);
    getInput("energy", energy);
    
    double new_energy = std::min(energy + energy_per_tick, max_energy);
    // std::cout << '[' << name() << "] " << "New energy: " << new_energy << std::endl;

    std_msgs::msg::Float64 energy_msg;
    energy_msg.data = new_energy;

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