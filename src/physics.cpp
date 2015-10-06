#include <vector>

#include "physics.hpp"

void Body::solve(double time_delta)
{
}

PhysicSimulation::PhysicSimulation() {
}

void PhysicSimulation::add_actor(Body *actor)
{
    this->m_actors.push_back(actor);
}


void PhysicSimulation::timestep(double time_delta)
{
    // apply forces
    for(std::vector<Body *>::iterator it = m_actors.begin(); it != m_actors.end(); ++it) {
	(*it)->apply_forces();
    }

    // integrate and update position
}
