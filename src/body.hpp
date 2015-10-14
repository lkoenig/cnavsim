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
    
protected:
    double _mass; // mass
    double _inertia;
    
    Vector2d _total_force;
    
    double _total_torque;
    
    double _heading;
    double _omega;    // rotational velocity
    double _omega_prime; 
    
    Vector2d _position;  // x, y, heading
    Vector2d _velocity;
    Vector2d _acceleration;
};

#endif
