#pragma once
#include "common.h"

class PIDController
{
public:
	PIDController(const double output_min, const double output_max, const double Kp, const double Ki, const double Kd)
		: output_min(output_min), output_max(output_max),
		Kp(Kp), Kd(Kd), Ki(Ki),
		integral(0),
		last_error(0), last_output(0) {};
	double Update(const double error, const double dt)
	{
		// Proportional term
		auto P = Kp * error;

		// Integral term
		integral += error * dt;
		auto I = Ki * integral;

		// Derivative term
		auto D = Kd * ((error - last_error) / dt);

		auto output = P + I + D;
		output = clamp(output, output_min, output_max);

		// Save terms
		last_output = output;
		last_error = error;

		return output;
	}
	double Query(const double error, const double dt)
	{
		const auto saved_last_error = last_error;
		const auto saved_last_output = last_output;
		const auto saved_integral = integral;
		const auto update = Update(error, dt);
		last_error = saved_last_error;
		last_output = saved_last_output;
		integral = saved_integral;
		return update;
	}
protected:
	double output_min, output_max;
	double Kp, Kd, Ki;
	double integral;
	double last_error, last_output;
};

class AntiWindupPIDController : public PIDController
{
public:
	AntiWindupPIDController(const double output_min, const double output_max, const double Kp, const double Ki, const double Kd)
		: PIDController(output_min, output_max, Kp, Ki, Kd) {};
	double Update(const double error, const double dt)
	{
		// Guard against integrator windup
		if ((last_output >= output_min && last_output <= output_max) || sign(error) != sign(last_output))
		{
			integral -= error * dt;
		}
		return PIDController::Update(error, dt);
	}
};