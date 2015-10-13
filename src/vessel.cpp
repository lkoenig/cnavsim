#include <iostream>
#include <Eigen/Dense>

#include "physics.hpp"
#include "vessel.hpp"

Vessel::Vessel()
: _length(12)
, _beam(4)
{
	_position << 0., 0.;
	_velocity << 5, 5;
	_mass = 10.; // grams
}

void Vessel::apply_forces()
{
    Vector2d velocity_in_water = _velocity;
    
	Vector2d gravity;  gravity <<  0, -10.;

	double C_D = 1;
	double A = 1;
	// F_d = 1/2 \rho v^2 C_D A
	// Where
	//   v is the velocity
	//   C_D coefficient of drag
	//   A Cross sectional area
    Vector2d drag_force = -1. / 2. * Constant::densityOfWater * _velocity.norm() * _velocity * C_D * A;

    
    // Rudder https://gamedev.stackexchange.com/questions/92747/2d-boat-controlling-physics
    
    
    Vector2d rudder_force; rudder_force << 0, 0;
    
    double rudder_torque = sin(_rudder_angle) * _rudder_surface * velocity_in_water.norm() * Constant::densityOfWater * _length / 2.;
    
    
    _total_torque = rudder_torque;
	_total_force = drag_force + gravity + rudder_force;

}
