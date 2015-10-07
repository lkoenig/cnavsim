#include <vector>
#include <iostream>

#include "physics.hpp"

void Body::time_step(double time_delta) {
	acceleration[0] = total_force[0] / mass;
	acceleration[1] = total_force[1] / mass;

	velocity[0] += acceleration[0] * time_delta;
	velocity[1] += acceleration[1] * time_delta;

	position[0] += velocity[0] * time_delta;
	position[1] += velocity[1] * time_delta;
}

void Body::print_position() {
	std::cout << position[0] << " " << position[1] << std::endl;
}


PhysicsEngine::PhysicsEngine() {
}

void PhysicsEngine::add_body(Body *actor)
{
    this->m_actors.push_back(actor);
}


void PhysicsEngine::timestep(double time_delta)
{
    // apply forces
    for(std::vector<Body *>::iterator it = m_actors.begin(); it != m_actors.end(); ++it) {
    	(*it)->apply_forces();
    }

    // integrate and update position
    for(std::vector<Body *>::iterator it = m_actors.begin(); it != m_actors.end(); ++it) {
    	(*it)->time_step(time_delta);
    }

    // integrate and update position
    for(std::vector<Body *>::iterator it = m_actors.begin(); it != m_actors.end(); ++it) {
    	(*it)->print_position();
    }

}
