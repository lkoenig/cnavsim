#ifndef BODY_H
#define BODY_H

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

class Body {
public:
    virtual void apply_forces() = 0;
    
    void time_step(double time_delta);

    Vector3d getGeneralizedPosition();
    Vector3d getGeneralizedVelocity();
    
protected:
	Body(
		double mass,
		double Iz,
		double xg
        );

    virtual ~Body() {};

    Vector2d m_gravityCenterPosition;
    
    Matrix<double, 3, 3> m_generalizedMass;
    Matrix<double, 3, 3> m_inverseGeneralizedMass;

    // Position in North East Down + Euler xyz
    Map< Vector3d > m_generalizedPosition; // N, E, psi
    Map< Vector2d > m_position; // N, E
    double *m_psi; // psi

    // Velocity in body fixed
    Map< Vector3d > m_generalizedVelocity;
    Map< Vector2d > m_linearVelocity; // U, V 
    double *m_r; // psi_dot

    Vector3d m_generalizedForce;

    double m_Iz;
    double m_mass; // mass
	double m_xg; // mass over xz plane (assumed yg = 0)
    
private:
    double m_generalizedPositionData[3]; // x, y, z, phy, theta, zeta
    double m_generalizedVelocityData[3]; // x, y, z, phy, theta, zeta

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
