#include <iostream>

#include "vessel.hpp"
#include "physics.hpp"

int main(int argc, char **argv) {
    std::cout << "NavSim" << std::endl;

    PhysicSimulation *physic = new PhysicSimulation();
    Vessel *vessel = new Vessel();

    physic->add_actor(vessel);
    physic->timestep(0.01);
    
    delete vessel;
    delete physic;    
}
