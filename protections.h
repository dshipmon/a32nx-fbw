#pragma once
#include "aircraft_data.h"
#include "input.h"

class NormalLawProtections
{
private:
	double max_bank_angle = 67; // Maximum bank angle in degrees
									// - Normally: 67 degrees
									// - High Angle of Attack Protection: 45 degrees
									// - High Speed Protection: 45 degrees
	double nominal_bank_angle = 33; // Spiral static stability in degrees
									// - Normally: 33 degrees
									// - High Angle of Attack Protection: 0 degrees
									// - High Speed Protection: 0 degrees
	double min_load_factor = -1; // -1g for clean configuration, 0g for other configurations
	double max_load_factor = 2.5; // 2.5g for clean configuration, 2g for other configurations
	double max_pitch_angle = 30; // Maximum pitch attitude in degrees
									 // 30 degrees nose up in conf 0-3 (progressively reduced to 25 degrees at low speed)
									 // 25 degrees nose up in conf FULL (progressively reduced to 20 degrees at low speed)
									 // TODO: To implement the 'progressively reduced' limitation, we need to find a way to
									 //       calculate speeds like V_alpha_prot or V_alpha_max, for which I think we will
									 //       have to work out via experimentation.
	double min_pitch_angle = -15; // Minimum pitch attitude in degrees

	bool aoa_demand_active = false;
	double aoa_demand_deactivation_timer = 0;
	bool high_speed_protection_active = false;

public:
	bool AoaDemandActive() { return aoa_demand_active; }
	bool HighSpeedProtActive() { return high_speed_protection_active; }
	double MaxBankAngle() { return max_bank_angle; }
	double MaxLoadFactor() { return max_load_factor; }
	double MaxPitchAngle() { return max_pitch_angle; }
	double MinLoadFactor() { return min_load_factor; }
	double MinPitchAngle() { return min_pitch_angle; }
	double NominalBankAngle() { return nominal_bank_angle; }
	
	void Update(const double t, const double dt)
	{
		// Check if we are in AoA demand mode (as dictated by the High Angle of Attack Protection)
		if (aoa_demand_active)
		{
			// Should we leave AoA demand mode?
			// Exit condition 1: Sidestick must be pushed more than 8 degrees forward (assuming this is ~50% down)
			const auto condition1 = input_capture.YokeY() <= -0.5;
			// Exit condition 2: Sidestick must be pushed more than 0.5 degrees forward for at least 0.5 seconds when alpha < alpha_max
			const auto condition2 = aoa_demand_deactivation_timer >= 0.5;
			if (condition1 || condition2)
			{
				aoa_demand_active = false;
				aoa_demand_deactivation_timer = 0;
			}
			else if (input_capture.YokeY() < 0 && aircraft_data.Alpha() < aircraft_data.AlphaMax())
			{
				// We're still building the target duration to meet condition 2
				aoa_demand_deactivation_timer += dt;
			}
			else
			{
				// We don't match any of the exit conditions
				aoa_demand_deactivation_timer = 0;
			}
		}
		else
		{
			// Should we enter AoA demand mode?
			// Enter condition 1: Sidestick must not be pushed down, and AoA is greater than alpha_prot
			const auto condition1 = input_capture.YokeY() >= 0 && aircraft_data.Alpha() > aircraft_data.AlphaProt();
			// Enter condition 2: We are at or above alpha max
			const auto condition2 = aircraft_data.Alpha() >= aircraft_data.AlphaMax();
			if (condition1 || condition2)
			{
				aoa_demand_active = true;
				aoa_demand_deactivation_timer = 0;
			}
		}

		// Check if high speed protection is active
		high_speed_protection_active = aircraft_data.IAS() > aircraft_data.Vmo()
		  	                        || aircraft_data.Mach() > aircraft_data.Mmo();

		// Update bank angle limits
		if (aoa_demand_active || high_speed_protection_active)
		{
			max_bank_angle = 45;
			nominal_bank_angle = 0;
		}
		else
		{
			max_bank_angle = 67;
			nominal_bank_angle = 33;
		}

		// Update load and pitch factors
		switch (aircraft_data.Flaps())
		{
		case 0: // Clean CONF
			min_load_factor = -1;
			max_load_factor = 2.5;
			max_pitch_angle = 30;
			break;
		case 1: // CONF 1
		case 2: // CONF 2
		case 3: // CONF 3
			min_load_factor = 0;
			max_load_factor = 2;
			max_pitch_angle = 30;
			break;
		case 4: // CONF FULL
			min_load_factor = 0;
			max_load_factor = 2;
			max_pitch_angle = 25;
			break;
		default: break;
		}
	}
};
NormalLawProtections normal_law_protections;