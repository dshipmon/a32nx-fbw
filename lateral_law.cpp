#include "lateral_law.h"

LateralLaw::LateralLaw()
{
	
}

double LateralLaw::Calculate(const double current_ailerons, const double current_rudder, const double t, const double dt)
{
	// Linked Linear System w/r roll-rate, lateral attitude, yaw rate, and side slip.
	// Control vec: Command yaw rate, command side slip.
	// TODO: Dynamic gain calculation.
	return -1;
}
