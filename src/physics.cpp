#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include "physics.hpp"
#include "body.hpp"

PhysicsEngine::PhysicsEngine()
    : m_current_time(0)
{
}

void PhysicsEngine::add_body(Body *actor)
{
    this->m_actors.push_back(actor);
}


void PhysicsEngine::timestep(double time_delta)
{
    m_current_time += time_delta;
    // apply forces
    for (std::vector<Body *>::iterator it = m_actors.begin(); it != m_actors.end(); ++it) {
        (*it)->apply_forces();
    }

    // integrate and update position
    for (std::vector<Body *>::iterator it = m_actors.begin(); it != m_actors.end(); ++it) {
        (*it)->time_step(time_delta);
    }
}

void PhysicsEngine::print_all_positions() {
    std::cout << m_current_time << " ";
    for (std::vector<Body *>::iterator it = m_actors.begin(); it != m_actors.end(); ++it) {
        std::cout << (*it)->getGeneralizedPosition().transpose() << " ";
        std::cout << (*it)->getGeneralizedVelocity().transpose() << " ";
    }
    std::cout << std::endl;

}
