#ifndef PHYSICS_H
#define PHYSICS_H

#include <vector>
#include <memory>
#include "body.hpp"

namespace Constant {
    static const double densityOfWater = 1.0;
}

class PhysicsEngine {
public:
    PhysicsEngine();
    
    void add_body(const std::shared_ptr<Body> actor);
    void timestep(double time_delta); 
    void print_all_positions();
    
    
private:
    double m_current_time;
    std::vector< std::shared_ptr<Body> > m_actors;
};

#endif /* PHYSICS_H */



