#ifndef PHYSICS_H
#define PHYSICS_H

#include <vector>

class Body {
public:
    
    void solve(double time_delta);
    virtual void apply_forces() = 0;

protected:
    double heading;
    double surge;
    double sway;
    double yaw;
    double mass;
};


class PhysicSimulation {
public:
    PhysicSimulation();
    
    void add_actor(Body *actor);
    void timestep(double time_delta); 
    
private:
    std::vector<Body *> m_actors;
};

#endif /* PHYSICS_H */



