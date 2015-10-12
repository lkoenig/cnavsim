#include <iostream>
#include <Eigen/Dense>
#include "body.hpp"

void Body::time_step(double time_delta) {
    _acceleration = _total_force / _mass;
    _velocity += _acceleration * time_delta;
    _position += _velocity * time_delta;
}

Vector2d Body::getPosition() {
    return _position;
}

Vector2d Body::getVelocity() {
    return _velocity;
}
