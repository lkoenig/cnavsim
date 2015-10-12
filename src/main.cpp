#include <iostream>

#include "vessel.hpp"
#include "physics.hpp"

int main(int argc, char **argv) {
    // std::cout << "NavSim" << std::endl;

    PhysicsEngine *physics = new PhysicsEngine();
    Vessel *vessel = new Vessel();

    physics->add_body(vessel);
    double time_delta = .001;
    
    for(double t = 0.0;t < 10.; t += time_delta) {
    	physics->timestep(time_delta);
        physics->print_all_positions();
    }

    delete vessel;
    delete physics;    
}
