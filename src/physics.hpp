#ifndef PHYSICS_H
#define PHYSICS_H

#include <vector>
#include <Eigen/Dense>

using namespace Eigen;

class Body {
public:
    virtual void apply_forces() = 0;

    void time_step(double time_delta);

    void print_position();

protected:
    double mass;

    Vector2d _total_force;

    Vector2d _position;
    Vector2d _velocity;
    Vector2d _acceleration;
};


class PhysicsEngine {
public:
    PhysicsEngine();
    
    void add_body(Body *actor);
    void timestep(double time_delta); 
    
private:
    std::vector<Body *> m_actors;
};

#endif /* PHYSICS_H */



