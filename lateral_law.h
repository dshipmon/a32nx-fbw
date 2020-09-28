#pragma once
class LateralLaw
{
public:
	LateralLaw();
	double Calculate(const double current_ailerons, const double current_rudder, const double t, const double dt);
private:
	// TODO: Add MATMUL, and Mat ADT.
};

