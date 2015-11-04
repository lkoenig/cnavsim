#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include "physics.hpp"
#include "body.hpp"

PhysicsEngine::PhysicsEngine()
    : m_current_time(0)
{
}

void PhysicsEngine::add_body(const std::shared_ptr<Body> actor)
{
    this->m_actors.push_back(actor);
}


void PhysicsEngine::timestep(double time_delta)
{
    m_current_time += time_delta;
    // apply forces
    for (auto& m_actor: m_actors) {
        m_actor->apply_forces();
    }

    // integrate and update position
    for (auto& m_actor : m_actors) {
        m_actor->time_step(time_delta);
    }
}

void PhysicsEngine::print_all_positions() {
    std::cout << m_current_time << " ";
    for (auto& m_actor : m_actors) {
        std::cout << m_actor->getGeneralizedPosition().transpose() << " ";
        std::cout << m_actor->getGeneralizedVelocity().transpose() << " ";
    }
    std::cout << std::endl;

}
