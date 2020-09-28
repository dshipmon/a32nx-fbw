#pragma once
#include "pid.h"
#include "pitch_control_mode.h"
#include "input.h"
#include "protections.h"

class RollController
{
private:
	double roll = 0; // The desired bank angle
	PIDController controller = PIDController(-1, 1, 0.10, 0, 0.02);
public:
	double Calculate(const double current_ailerons, const double t, const double dt)
	{
		// TODO: Handle other control laws besides normal law

		if (pitch_control_mode.Mode() == FLIGHT_MODE || pitch_control_mode.Mode() == FLARE_MODE)
		{
			if (input_capture.YokeX() == 0)
			{
				// If we are banked beyond the nominal bank angle, roll back to the nominal bank angle
				if (fabs(roll) > normal_law_protections.NominalBankAngle())
				{
					// Roll opposite at a rate of up to 5 degrees/second
					roll += 5 * -sign(roll) * dt;
					if (fabs(roll) < normal_law_protections.NominalBankAngle())
					{
						roll = sign(roll) * normal_law_protections.NominalBankAngle();
					}
				}
				// We should be holding the specified roll angle
			}
			else
			{
				// We should be responsive to the user's roll request
				roll += 15 * input_capture.YokeX() * dt; // 15 degrees/sec at maximum deflection
				roll = clamp(roll, -normal_law_protections.MaxBankAngle(), normal_law_protections.MaxBankAngle());
			}
			return controller.Update(roll - aircraft_data.Roll(), dt);
		}
		else
		{
			// Ground mode
			return input_capture.RawYokeX();
		}
	}
};