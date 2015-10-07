#ifndef PHYSICS_H
#define PHYSICS_H

#include <vector>

class Body {
public:
    virtual void apply_forces() = 0;
    void time_step(double time_delta);
    void print_position();

protected:
    double mass;
    double total_force[2];

    double position[2];
    double velocity[2];
    double acceleration[2];
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



