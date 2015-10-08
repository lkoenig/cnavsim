#include <iostream>
#include <Eigen/Dense>

#include "vessel.hpp"


Vessel::Vessel() {
	_position << 0., 0.;
	_velocity << 5, 5;
	mass = 10.; // grams
}

void Vessel::apply_forces()
{
	Vector2d gravity;  gravity <<  0, -10.;

	double C_D = 1;
	double A = 1;
	double rho = 1000;
	// F_d = 1/2 \rho v^2 C_D A
	// Where \rho is density of water
	//   v is the velocity
	//   C_D coefficient of drag
	//   A Cross sectional area
	Vector2d drag_force = -1. / 2. * rho * _velocity.norm() * _velocity * C_D * A;

	_total_force = drag_force + gravity;

}
