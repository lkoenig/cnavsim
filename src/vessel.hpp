#include "body.hpp"

class Vessel : public Body {
public:
	Vessel();

    virtual void apply_forces();
    
    virtual double getLength(){return m_length;};
    virtual double getBeam() {return m_beam;};

private:
    double rudder_lift_coefficient();
    double rudder_drag_coefficient();


    // https://en.wikipedia.org/wiki/Ship_measurements
    double m_length; // in m
    double m_beam; // in m
    
    // Parameters
    double m_rudder_angle; // in degree
    double m_rudder_area; // in m**2
};
