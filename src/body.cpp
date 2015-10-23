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
    Matrix3d C21 = -m_mass * (SkewSymetric(m_linearVelocity) + SkewSymetric(SkewSymetric(m_angularVelocity) * m_gravityCenterPosition));
    Matrix3d C22 = m_mass * SkewSymetric(SkewSymetric(m_linearVelocity) * m_gravityCenterPosition) - SkewSymetric(m_inertia * m_angularVelocity);
    Matrix<double, 6, 6> coriolis;
    coriolis << 
        MatrixXd::Zero(3, 3), C21,
        C21, C22;

    Matrix<double, 6, 1> momentum = m_inverseGeneralizedMass * (m_generalizedForce - coriolis * m_generalizedVelocity);
    m_generalizedVelocity += time_delta * momentum;
	
    m_generalizedPosition += time_delta * Rotation(m_angle) * m_generalizedVelocity;


}

Matrix<double, 6, 1> Body::getGeneralizedPosition() {
    return m_generalizedPosition;
}

Matrix<double, 6, 1> Body::getGeneralizedVelocity() {
    return m_generalizedVelocity;
}

Body::Body(
    double mass,
    Matrix3d inertia
    )
    : m_gravityCenterPosition(Vector3d::Zero())
    , m_generalizedPosition(m_generalizedPositionData, 6)
    , m_position(m_generalizedPositionData, 3)
    , m_angle(&m_generalizedPositionData[3], 3)
    , m_generalizedVelocity(m_generalizedVelocityData, 6)
    , m_linearVelocity(m_generalizedVelocityData, 3)
    , m_angularVelocity(&m_generalizedVelocityData[3], 3)
    , m_generalizedForce(MatrixXd::Zero(6,1))
    , m_inertia(inertia)
    , m_mass(mass)
{
    m_generalizedMass << 
        m_mass * MatrixXd::Identity(3,3), -m_mass * SkewSymetric(m_gravityCenterPosition), 
        m_mass * SkewSymetric(m_gravityCenterPosition), m_inertia;

    m_inverseGeneralizedMass = m_generalizedMass.inverse();

    memset(m_generalizedPositionData, 0, sizeof(m_generalizedPositionData));
    memset(m_generalizedVelocityData, 0, sizeof(m_generalizedVelocityData));

}
