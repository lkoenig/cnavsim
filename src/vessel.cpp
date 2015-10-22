#include <iostream>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

#include "physics.hpp"
#include "vessel.hpp"

Vessel::Vessel()
    : Body(10., MatrixXd::Identity(3,3))
    , m_length(12)
    , m_beam(4)
{
    m_linearVelocity << 5, 5, 0;
    m_rudder_angle = 10 * 2 * 3.14 / 360.;
    m_rudder_area = 1.0;
}

void Vessel::apply_forces()
{
	// http://www.marinecontrol.org/

    // Rudder
    double rudder_effective_area = m_rudder_area * sin(m_rudder_angle);

    double rudder_lift = 0.5 * Constant::densityOfWater * rudder_effective_area * m_linearVelocity.norm() * m_linearVelocity.norm() * rudder_lift_coefficient();
    double rudder_drag = 0.5 * Constant::densityOfWater * rudder_effective_area * m_linearVelocity.norm() * m_linearVelocity.norm() * rudder_drag_coefficient();

    Vector3d rudder_linear = rudder_lift * Vector3d::UnitX() - rudder_drag * Vector3d::UnitY();


	Matrix<double, 6, 1> rudder;
	rudder << rudder_linear, -m_length / 2 * Vector3d::UnitY().cross(rudder_linear);

    double C_D = 1;
    double A = 1;
    // F_d = 1/2 \rho v^2 C_D A
    // Where
    //   v is the velocity
    //   C_D coefficient of drag
    //   A Cross sectional area
	Matrix<double, 6, 1> drag_force;
	drag_force << 
		- 0.5 * Constant::densityOfWater * m_linearVelocity.norm() * m_linearVelocity * C_D * A,
		Vector3d::Zero();

	m_generalizedForce = rudder + drag_force;
}

double Vessel::rudder_lift_coefficient()
{
    return 1.0;
}

double Vessel::rudder_drag_coefficient()
{
    return 1.0;
}
