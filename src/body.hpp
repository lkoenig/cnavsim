#ifndef BODY_H
#define BODY_H

#include <Eigen/Dense>

using namespace Eigen;

class Body {
public:
    virtual void apply_forces() = 0;
    
    void time_step(double time_delta);

	Matrix<double, 6, 1> getGeneralizedPosition();
	Matrix<double, 6, 1> getGeneralizedVelocity();
    double getHeading();
    
protected:
    Body(
        double mass,
        Matrix3d inertia
        );

    virtual ~Body() {};

    Vector3d m_gravityCenterPosition;
    
    Matrix<double, 6, 6> m_generalizedMass;
    Matrix<double, 6, 6> m_inverseGeneralizedMass;

    // Position
    Map<Matrix<double, 6, 1>> m_generalizedPosition; // N, E, D, phy, theta, psi
    Map<Vector3d> m_position; // N, E, D
    Map<Vector3d> m_angle; // phy, theta, psi

    // Velocity
    Map<Matrix<double, 6, 1>> m_generalizedVelocity;
    Map<Vector3d> m_linearVelocity; // N, E, D
    Map<Vector3d> m_angularVelocity; // phy, theta, psi

    Matrix<double, 6, 1> m_generalizedForce;

    Matrix3d m_inertia;
    double m_mass; // mass
    
private:
    double m_generalizedPositionData[6]; // x, y, z, phy, theta, zeta
    double m_generalizedVelocityData[6]; // x, y, z, phy, theta, zeta

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
