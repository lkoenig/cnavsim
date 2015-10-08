#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include "physics.hpp"

void Body::time_step(double time_delta) {
	_acceleration = _total_force / mass;
	_velocity += _acceleration * time_delta;
	_position += _velocity * time_delta;
}

void Body::print_position() {
	std::cout << _position.transpose() << std::endl;
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
