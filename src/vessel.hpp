#include "body.hpp"

class Vessel : public Body {
public:
	Vessel();

    virtual void apply_forces();
    
private:
    double rudder_lift_coefficient();
    double rudder_drag_coefficient();


    // https://en.wikipedia.org/wiki/Ship_measurements
    double _length; // in m
    double _beam; // in m
    
    // Parameters
    double _rudder_angle; // in degree
    double _rudder_area; // in m**2
};
