#include <iostream>

#include "vessel.hpp"
#include "physics.hpp"

int main(int argc, char **argv) {
    // std::cout << "NavSim" << std::endl;

    PhysicsEngine *physics = new PhysicsEngine();
    Vessel *vessel = new Vessel();

    physics->add_body(vessel);
    double time_delta = .001; // 0.1 should be sufficient

    for (double t = 0.0; t < 200.; t += time_delta) {
        physics->timestep(time_delta);
        physics->print_all_positions();
    }

    delete vessel;
    delete physics;
}
