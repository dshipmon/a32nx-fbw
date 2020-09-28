#pragma once
#include "aircraft_data.h"
#include "protections.h"
#include "pid.h"
#include "pitch_control_mode.h"
#include "common.h"

class PitchController
{
private:
	// Controllers
	AntiWindupPIDController aoa_controller = AntiWindupPIDController(-2, 2, 0.002, 0, 0.0002); // AoA error -> elevator handle movement rate
	AntiWindupPIDController gforce_controller = AntiWindupPIDController(-2, 2, 0.008, 0.008, 0.001); // GForce error -> elevator handle movement rate
	AntiWindupPIDController vertical_fpa_controller = AntiWindupPIDController(-2, 2, 0.0015, 0.0020, 0.002); // Vertical FPA error -> elevator handle movement rate
	AntiWindupPIDController pitch_rate_controller = AntiWindupPIDController(-2, 2, 0.01, 0.015, 0.0025); // Pitch rate error -> elevator handle movement rate

	double held_pitch_time = 0;
	double held_vertical_fpa = 0;

	// Applies load factor limitation protection to a proposed elevator movement
	double LoadFactorLimitation(const double delta_elevator, const double dt)
	{
		if (aircraft_data.GForce() > normal_law_protections.MaxLoadFactor())
		{
			const auto new_delta_elevator = gforce_controller.Update(normal_law_protections.MaxLoadFactor() - aircraft_data.GForce(), dt);
			printf(",LF_LIMIT_MAX:PreDE=%lf,PostDE=%lf", delta_elevator, new_delta_elevator);
			return new_delta_elevator;
		}

		if (aircraft_data.GForce() < normal_law_protections.MinLoadFactor())
		{
			const auto new_delta_elevator = gforce_controller.Update(normal_law_protections.MinLoadFactor() - aircraft_data.GForce(), dt);
			printf(",LF_LIMIT_MIN:PreDE=%lf,PostDE=%lf", delta_elevator, new_delta_elevator);
			return new_delta_elevator;
		}
		
		return delta_elevator;
	}

	// Applies high speed protection to a proposed elevator movement
	double HighSpeedProtection(const double delta_elevator, const double dt)
	{
		if (!normal_law_protections.HighSpeedProtActive()) return delta_elevator;

		held_pitch_time = 0;
		
		double user = 0;
		if (user < 0)
		{
			// The FCOM says "As the speed increases above VMO/MMO, the sidestick nose-down authority is progressively reduced"
			// Let's make the user have no authority above Vmo + 8, Mmo + 0.012 (arbitrarily chosen)
			// We'll pick whichever path leaves the user with the least control
			const auto user_knots = user * linear_decay_coefficient(aircraft_data.IAS(), aircraft_data.Vmo(), aircraft_data.Vmo() + 8);
			const auto user_mach = user * linear_decay_coefficient(aircraft_data.Mach(),aircraft_data.Mmo(), aircraft_data.Mmo() + 0.012);
			user = fmax(user_knots, user_mach);
		}

		// Now let's get the nose-up input necessary
		// We'll aim the speed for Vmo - 1, Mmo - 0.0015 (arbitrarily chosen)
		// We'll cap at a maximum of 5 degrees/second pitch-up as the speed goes past Vmo + 16 degree, Mmo + 0.024 (arbitrarily chosen)
		const auto recovery_pitch_rate_knots = 5 * linear_decay_coefficient(aircraft_data.IAS(), aircraft_data.Vmo() + 16, aircraft_data.Vmo() - 1);
		const auto recovery_pitch_rate_mach = 5 * linear_decay_coefficient(aircraft_data.Mach(), aircraft_data.Mmo() + 0.024, aircraft_data.Mmo() - 0.0015);
		const auto recovery_pitch_rate = fmax(recovery_pitch_rate_knots, recovery_pitch_rate_mach);
		const auto recovery = pitch_rate_controller.Update(recovery_pitch_rate - aircraft_data.PitchRate(), dt);

		// Let's blend the two together
		const auto new_delta_elevator = user + recovery;
		printf(",OVSPD:PreDE=%lf,UserDE=%lf,RecDE=%lf,PostDE=%lf", delta_elevator, user, recovery, new_delta_elevator);
		return new_delta_elevator;
	}

	// Applies pitch attitude protection to a proposed elevator movement
	double PitchAttitudeProtection(const double delta_elevator, const double dt)
	{	
		if (aircraft_data.Pitch() > normal_law_protections.MaxPitchAngle())
		{
			// Correct using up to -5 degrees/second pitch rate when we are up to 1 degree above our limit
			// Thereafter, correct using -5 degrees/second pitch rate
			const auto corrective_pitch_rate = -5 * linear_decay_coefficient(aircraft_data.Pitch(), normal_law_protections.MaxPitchAngle() + 1, normal_law_protections.MaxPitchAngle());
			const auto new_delta_elevator = pitch_rate_controller.Update(corrective_pitch_rate - aircraft_data.PitchRate(), dt);
			printf(",MAX_P_VIOL:PreDE=%lf,DesPR=%lf,PostDE=%lf", delta_elevator, corrective_pitch_rate, new_delta_elevator);
			return new_delta_elevator;
			
		}
		if (aircraft_data.Pitch() < normal_law_protections.MinPitchAngle())
		{
			// Correct using up to +5 degrees/second pitch rate when we are up to 1 degree above our limit
			// Thereafter, correct using +5 degrees/second pitch rate
			const auto corrective_pitch_rate = 5 * linear_decay_coefficient(aircraft_data.Pitch(), normal_law_protections.MinPitchAngle() - 1, normal_law_protections.MinPitchAngle());
			const auto new_delta_elevator = pitch_rate_controller.Update(corrective_pitch_rate - aircraft_data.PitchRate(), dt);
			printf(",MIN_P_VIOL:PreDE=%lf,DesPR=%lf,PostDE=%lf", delta_elevator, corrective_pitch_rate, new_delta_elevator);
			return new_delta_elevator;
		}
		
		// Naturally limit the pitch up/down rate from +/-30 degree/sec to 0 as we approach our limits
		const auto max_pitch_rate = 30 * linear_decay_coefficient(aircraft_data.Pitch(), 0, normal_law_protections.MaxPitchAngle());
		if (aircraft_data.PitchRate() > max_pitch_rate && delta_elevator >= 0)
		{
			const auto new_delta_elevator = pitch_rate_controller.Update(max_pitch_rate - aircraft_data.PitchRate(), dt);
			printf(",PR_LIM_MAX:PreDE=%lf,MaxPR=%lf,PostDE=%lf", delta_elevator, max_pitch_rate, new_delta_elevator);
			return new_delta_elevator;
		}
		
		const auto min_pitch_rate = -30 * linear_decay_coefficient(aircraft_data.Pitch(), 0, normal_law_protections.MinPitchAngle());
		if (aircraft_data.PitchRate() < min_pitch_rate && delta_elevator <= 0)
		{
			const auto new_delta_elevator = pitch_rate_controller.Update(min_pitch_rate - aircraft_data.PitchRate(), dt);
			printf(",PR_LIM_MAX:PreDE=%lf,MinPR=%lf,PostDE=%lf", delta_elevator, min_pitch_rate, new_delta_elevator);
			return new_delta_elevator;
		}

		return delta_elevator;
	}

	// Applies rules assuming sidestick demands angle of attack
	double AngleOfAttackDemand(const double dt)
	{
		held_pitch_time = 0;

		const auto commanded_aoa = input_capture.YokeY() >= 0 ?
			  // Neutral -> Full Up = AoA proportional range from alpha_prot -> alpha_max
			  linear_range(input_capture.YokeY(), aircraft_data.AlphaProt(), aircraft_data.AlphaMax())
			  // Neutral -> Full Down = AoA proportional range from alpha_prot -> 0 AoA
			: linear_range(input_capture.YokeY(), aircraft_data.AlphaProt(), 0);

		auto delta_elevator = aoa_controller.Update(commanded_aoa - aircraft_data.Alpha(), dt);
		printf("AOA:AOA=%lf,DesAOA=%lf,ErrAOA=%lf", aircraft_data.Alpha(), commanded_aoa,  commanded_aoa - aircraft_data.Alpha());

		// Apply protections
		delta_elevator = LoadFactorLimitation(delta_elevator, dt);
		delta_elevator = PitchAttitudeProtection(delta_elevator, dt); // This isn't specified in the FCOM, but the flight model is not true enough to real life
		
		return delta_elevator;
	}

	// Applies rules assuming sidestick demands load factor
	double LoadFactorDemand(const double dt)
	{
		double delta_elevator;
		if (input_capture.YokeX() == 0 && input_capture.YokeY() == 0)
		{
			// Neutral x and y = Hold FPA
			if (held_pitch_time < 5)
			{
				// Hold the current pitch for 5 seconds to allow VFPA to stabilize
				delta_elevator = pitch_rate_controller.Update(0 - aircraft_data.PitchRate(), dt);
				held_vertical_fpa = aircraft_data.VFPA();
				held_pitch_time += dt;
				printf("HOLD_PITCH:");
			}
			else
			{
				// Hold the VFPA				
				delta_elevator = vertical_fpa_controller.Update(held_vertical_fpa - aircraft_data.VFPA(), dt);
				printf("HOLD_VFPA:DesVFPA=%lf", held_vertical_fpa);
			}
		}
		else if (input_capture.YokeY() == 0 && fabs(aircraft_data.Roll() > normal_law_protections.NominalBankAngle()))
		{
			held_pitch_time = 0;
			
			// Neutral y, but we're rolling and bank angle is greater than our nominal bank angle = Drop pitch to 1G LF
			delta_elevator = gforce_controller.Update(1 - aircraft_data.GForce(), dt);
			printf("ROLL_1G:");
		}
		else if (input_capture.YokeY() == 0)
		{
			held_pitch_time = 0;
			
			// Neutral y, but we're rolling and bank angle is less than our nominal bank angle = Hold pitch
			delta_elevator = pitch_rate_controller.Update(0 - aircraft_data.PitchRate(), dt);
			printf("HOLD_PITCH:");
		}
		else
		{
			// Both x and y input
			held_pitch_time = 0;

			// Determine the normal load factor for our bank angle
			const auto normal_load_factor = 1 / cos(radians(aircraft_data.Roll()));

			// Determine the user's requested load factor
			const auto requested_load_factor = input_capture.YokeY() >= 0 ?
				  linear_range(input_capture.YokeY(), normal_load_factor, normal_law_protections.MaxLoadFactor())
				: linear_range(-input_capture.YokeY(), normal_load_factor, normal_law_protections.MinLoadFactor());

			delta_elevator = gforce_controller.Update(requested_load_factor - aircraft_data.GForce(), dt);
			printf("CMD_LF:NLF=%lf,RLF=%lf,LFErr=%lf", normal_load_factor, requested_load_factor, requested_load_factor - aircraft_data.GForce());
		}

		// Apply protections
		delta_elevator = HighSpeedProtection(delta_elevator, dt);
		delta_elevator = LoadFactorLimitation(delta_elevator, dt);
		delta_elevator = PitchAttitudeProtection(delta_elevator, dt);
		
		return delta_elevator;
	}

	
	double FlareModeDemand(const double dt)
	{
		// Let's make the sidestick action at flare mode just a pitch rate mode for simplicity's sake
		auto pitch_rate = 5 * input_capture.YokeY(); // 5 degrees/sec at maximum deflection
		if (aircraft_data.RadioHeight() <= 30)
		{
			// From the FCOM
			// "The system memorizes the attitude at 50 feet, and that attitude becomes the initial
			// reference for pitch attitude control."
			// We saved that variable as 'saved_flare_pitch_attitude'.
			
			// From the FCOM
			// "As the aircraft descends through 30 feet, the system begins to reduce the pitch attitude,
			// reducing it to 2 degrees nose down over a period of 8 seconds. This means that it takes
			// gentle nose-up action by the pilot to flare the aircraft."
			
			// My interpretation is that the plane will try to dip to 2 degrees below the saved flare pitch attitude -- Not aggressive enough
			// My new interpretation is that the plane will try to dip to 2 degrees below the horizon 

			pitch_rate += ((-2) - aircraft_data.Pitch()) / 8;
		}

		const auto delta_elevator = pitch_rate_controller.Update(pitch_rate - aircraft_data.PitchRate(), dt);
		printf("FLARE:DesPR=%lf,RH=%lf", pitch_rate, aircraft_data.RadioHeight());
		return delta_elevator;
	}
public:
	
	double Calculate(const double current_elevator, const double t, const double dt)
	{
		// On the ground, pitch is direct
		// TODO: Add ground mode calculations (e.g. when aircraft reaches 70 knots during the T/O roll, maximum deflection of elevators is affected)

		double new_elevator;
		if (pitch_control_mode.Mode() == GROUND_MODE) new_elevator = input_capture.RawYokeY();

		// AoA protections are available in both flight/flare modes
		else if (normal_law_protections.AoaDemandActive()) new_elevator = current_elevator + AngleOfAttackDemand(dt);

		// Flare mode has a special effect and does not have all the protections of flight mode
		else if (pitch_control_mode.FlareEffect() > 0) new_elevator = current_elevator + FlareModeDemand(dt);

		// Flight mode
		else new_elevator = current_elevator + LoadFactorDemand(dt);

		new_elevator = clamp(new_elevator, -1, 1);
		printf(",P=%lf,PR=%lf,VFPA=%lf,VFPAR=%lf,LF=%lf,DE=%lf,E=%lf\n",
			aircraft_data.Pitch(), aircraft_data.PitchRate(),
			aircraft_data.VFPA(), aircraft_data.VFPARate(),
			aircraft_data.GForce(),
			new_elevator - current_elevator, new_elevator);
		return new_elevator;
	}
};