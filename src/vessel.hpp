#include "physics.hpp"

class Vessel : public Body {
public:
	Vessel();

    virtual void apply_forces();
};
