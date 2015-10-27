#include <iostream>
#include <Eigen/Dense>
#include "body.hpp"

Matrix3d SkewSymetric(Vector3d x)
{
    Matrix3d S;
    S << 0, -x(2), x(1),
        x(2), 0, -x(0),
        -x(1), x(0), 0;
    return S;
}

Matrix<double, 6, 6> Rotation(Vector3d eulerAngle) 
{
    Matrix3d R;
    double phi = eulerAngle(0);
    double theta = eulerAngle(1);
    double psi = eulerAngle(2);

    double cphi = cos(phi);
    double ctheta = cos(theta);
    double cpsi = cos(psi);

    double sphi = sin(phi);
    double spsi = sin(psi);
    double stheta = sin(theta);
	
    R <<
	cpsi * ctheta,     -spsi * cphi + cpsi * stheta * sphi,     spsi * sphi + cpsi * cphi * stheta,
	spsi * ctheta,      cpsi * cphi + sphi * stheta * spsi,    -cpsi * sphi + spsi * cphi * stheta,
	-stheta,                           ctheta * sphi,                          ctheta * cphi;

    Matrix3d J2;
    J2 <<
	1, sphi * stheta / ctheta, cphi * stheta / ctheta,
	0, cphi, -sphi,
	0, sphi / ctheta, cphi / ctheta;
    
    Matrix<double, 6, 6> J;
    J << 
	R, Matrix3d::Zero(),
	Matrix3d::Zero(), J2;

    return J;

}

void Body::time_step(double time_delta) 
{
	Vector3d momentum = m_inverseGeneralizedMass * m_generalizedForce;

	// Runge kutta Assuming force are equals
	//Matrix<double, 6, 1> k1, k2, k3, k4;
	//k1 = m_generalizedVelocity + time_delta * momentum;
	//k2 = m_generalizedVelocity + time_delta * 0.5 * k1;
	//k3 = m_generalizedVelocity + time_delta * 0.5 * k2;
	//k4 = m_generalizedVelocity + time_delta * k3;
	//m_generalizedVelocity += time_delta / 6. * (k1 + 2. * k2 + 2. * k3 + k4);

	m_generalizedVelocity += time_delta * momentum;

	Matrix3d J;
	double cpsi = cos(*m_psi);
	double spsi = sin(*m_psi);
	J <<
		cpsi, -spsi, 0,
		spsi, cpsi, 0,
		0, 0,  1;

    m_generalizedPosition += time_delta * J * m_generalizedVelocity;


}

Vector3d Body::getGeneralizedPosition() {
    return m_generalizedPosition;
}

Vector3d Body::getGeneralizedVelocity() {
    return m_generalizedVelocity;
}

Body::Body(
    double mass,
    double Iz,
	double xg
    )
    : m_gravityCenterPosition(Vector2d::Zero())
    , m_generalizedPosition(m_generalizedPositionData, sizeof(m_generalizedPositionData) / sizeof(m_generalizedPositionData[0]))
    , m_position(m_generalizedPositionData, 2)
    , m_psi(&m_generalizedPositionData[2])
    , m_generalizedVelocity(m_generalizedVelocityData, sizeof(m_generalizedVelocityData) / sizeof(m_generalizedVelocityData[0]))
    , m_linearVelocity(m_generalizedVelocityData, 2)
    , m_r(&m_generalizedVelocityData[2])
    , m_generalizedForce(Vector3d::Zero())
    , m_Iz(Iz)
    , m_mass(mass)
	, m_xg(xg)
{
	m_generalizedMass <<
		m_mass, 0, 0,
		0, m_mass, m_mass * m_xg,
		0, m_mass * m_xg, m_Iz;

    m_inverseGeneralizedMass = m_generalizedMass.inverse();

    memset(m_generalizedPositionData, 0, sizeof(m_generalizedPositionData));
    memset(m_generalizedVelocityData, 0, sizeof(m_generalizedVelocityData));

}
