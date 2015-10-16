#include <iostream>
#include <Eigen/Dense>
#include "body.hpp"

void Body::time_step(double time_delta) {
    _acceleration = _total_force / _mass;
    _velocity += _acceleration * time_delta;
    _position += _velocity * time_delta;

    m_angularMomentum = _total_torque / _inertia;
    m_angularVelocity += m_angularMomentum * time_delta;
    _heading += m_angularVelocity * time_delta;

}

Vector2d Body::getPosition() {
    return _position;
}

Vector2d Body::getVelocity() {
    return _velocity;
}

double Body::getHeading() {
    return _heading;
}

void Body::addForceAt(Vector2d *force, Vector2d *Position)
{
    // Update force on center of mass

    // update torque

    return;
}
