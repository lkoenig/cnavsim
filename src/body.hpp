#ifndef BODY_H
#define BODY_H

#include <Eigen/Dense>

using namespace Eigen;

class Body {
public:
    virtual void apply_forces() = 0;
    
    void time_step(double time_delta);

    Vector2d getPosition();
    Vector2d getVelocity();
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


    // Old stuff
    Vector2d _total_force;

    Vector2d _position;  // x, y, heading
    Vector2d _velocity;
    Vector2d _acceleration;


    Matrix3d m_inertia;
    double m_mass; // mass
    
    
    double _inertia;
        
    double _total_torque;
    
    double _heading;
    double _angularVelocity;    // rotational velocity
    double _angularMomentum; 

private:
    double m_generalizedPositionData[6]; // x, y, z, phy, theta, zeta
    double m_generalizedVelocityData[6]; // x, y, z, phy, theta, zeta

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
