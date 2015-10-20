#include <iostream>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

#include "physics.hpp"
#include "vessel.hpp"

Vessel::Vessel()
    : Body(10., MatrixXd::Identity(3,3))
    , _length(12)
    , _beam(4)
{
    m_linearVelocity << 5, 5, 0;
    _heading = 0;
    _angularVelocity = 0;
    _position << 0., 0.;
    _velocity << 5, 5;
    _inertia = 1;
    _rudder_angle = 10 * 2 * 3.14 / 360.;
    _rudder_area = 1.0;
}

void Vessel::apply_forces()
{
    Vector2d velocity_in_water = _velocity;

    double C_D = 1;
    double A = 1;


    // F_d = 1/2 \rho v^2 C_D A
    // Where
    //   v is the velocity
    //   C_D coefficient of drag
    //   A Cross sectional area
    Vector2d drag_force = -1. / 2. * Constant::densityOfWater * _velocity.norm() * _velocity * C_D * A;


    // Rudder https://gamedev.stackexchange.com/questions/92747/2d-boat-controlling-physics
    double rudder_drag_coefficient = 1.;
    Vector2d rudder_drag_force;
    rudder_drag_force = -std::abs(sin(_rudder_angle)) * rudder_drag_coefficient * _rudder_area * _velocity.norm() * _velocity;

    double turning_torque = sin(_rudder_angle) * _rudder_area * velocity_in_water.norm() * Constant::densityOfWater * _length / 2.;
    double water_torque = .1; // p0 + p1 * m_angularVelocity + p2 * m_angularVelocity * m_angularVelocity;
    double rudder_torque = copysign(std::max(0.0, std::abs(turning_torque) - std::abs(water_torque)), turning_torque);

    _total_torque = rudder_torque;
    _total_force = drag_force + rudder_drag_force;

}
