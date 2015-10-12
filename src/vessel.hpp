#include "body.hpp"

class Vessel : public Body {
public:
	Vessel();

    virtual void apply_forces();
    
private:
    // https://en.wikipedia.org/wiki/Ship_measurements
    double _length; // in m
    double _beam; // in m
    
    // Parameters
    double _rudder_angle; // in degree
};
