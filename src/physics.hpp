#ifndef PHYSICS_H
#define PHYSICS_H

#include <vector>
#include "body.hpp"

namespace Constant {
    static const double densityOfWater = 1.0;
}

class PhysicsEngine {
public:
    PhysicsEngine();
    
    void add_body(Body *actor);
    void timestep(double time_delta); 
    void print_all_positions();
    
    
private:
    double _current_time;
    std::vector<Body *> m_actors;
};

#endif /* PHYSICS_H */



