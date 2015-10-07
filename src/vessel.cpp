#include <iostream>

#include "vessel.hpp"


Vessel::Vessel() {
	position[0] = 0.;  position[1] = 0.;
	velocity[0] = 5;  velocity[1] = 5;
	mass = 10.;
}

void Vessel::apply_forces()
{
	total_force[0] = 0;
	total_force[1] = -10.;

}
