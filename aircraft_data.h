#pragma once
#include <cmath>

#include "common.h"

class AircraftData
{
private:

	double aoa = 0; // The angle of attack in degrees
	bool autopilot = false; // True if the autopilot is on
	int flaps = 0; // The current position of the flaps handle (0 = Clean CONF, 4 = CONF FULL)
	double gforce = 0; // The current gforce (load factor)
	double ias = 0; // The indicated airspeed in knots
	double lateral_speed = 0; // Lateral speed (relative to the earth in a north/south direction) in feet/second
	double longitudinal_speed = 0; // Longitudinal speed (relative to the earth in an east/west direction) in feet/second
	double mach = 0; // The current speed in mach
	double mmo = DBL_MAX; // The Mmo speed in mach
	bool on_ground = true; // True if the plane is on the ground
	double pitch = 0; // Pitch attitude in degrees (+ is up, - is down)
	double pitch_rate = 0; // Pitch attitude rate in degrees/sec (+ is up, - is down)
	double radio_height = 0; // Radio altimeter in feet
	double roll = 0; // Roll attitude in degrees (+ is right, - is left)
	double vertical_speed = 0; // Vertical speed (relative to the earth) in feet/second
	double vfpa_rate = 0; // Vertical flight path angle rate in degrees/second
	double vmo = DBL_MAX; // The Vmo speed in knots

	double last_pitch = 0;
	double last_vfpa = 0;

	double FetchSimVar(const char * name, const char * units, const int index, const double fallback)
	{
		const auto value = aircraft_varget(get_aircraft_var_enum(name), get_units_enum(units), index);
		return isnan(value) ? fallback : value;
	}
public:

	double Alpha() { return aoa; }
	double AlphaFloor()
	{
		// These values are hardcoded in the FCOM in 1.27.20 under "High Angle of Attack Protection"
		// Note: 2. a.floor is activated through A/THR system when:
		// - a > a floor (9.5 degrees in configuration 0; 15 degrees in configuration 1, 2; 14 degrees in
		//   configuration 3; 13 degrees in configuration FULL), or,...
		// TODO: These values don't seem to mesh well with how the A320 is actually modeled, even though
		//       they come from the manual directly.
		switch (flaps)
		{
		case 0: // Clean CONF
			return 9.5;
			break;
		case 1: // CONF 1
		case 2: // CONF 2
			return 15;
			break;
		case 3: // CONF 3
			return 14;
			break;
		case 4: // 
			return 13;
			break;
		default: // Unreachable
			return 9.5;
			break;
		}
	}
	double AlphaProt()
	{
		// This ratio was estimated using the graph in the FCOM in 1.27.20 under "High Angle of Attack Protection"
		// The graph plots CL (lift coefficient) to alpha.
		// The ratio was guesstimated using a ruler and hoping the graph was accurate.
		const auto ratio_with_alpha_floor = 19.0 / 21.0;
		return ratio_with_alpha_floor * AlphaFloor();
	}
	double AlphaMax()
	{
		// This ratio was estimated using the graph in the FCOM in 1.27.20 under "High Angle of Attack Protection"
		// The graph plots CL (lift coefficient) to alpha.
		// The ratio was guesstimated using a ruler and hoping the graph was accurate.
		const auto ratio_with_alpha_floor = 7.0 / 6.0;
		return ratio_with_alpha_floor * AlphaFloor();
	}
	bool Autopilot() { return autopilot; }
	int Flaps() { return flaps; }
	double GForce() { return gforce;  }
	double IAS() { return ias; }
	double Mach() { return mach; }
	double Mmo() { return mmo; }
	bool OnGround() { return on_ground; }
	double Pitch() { return pitch; }
	double PitchRate() { return pitch_rate; }
	double RadioHeight() { return radio_height; }
	double Roll() { return roll;  }
	double VFPA()
	{
		const auto horizontal_speed = sqrt(lateral_speed * lateral_speed + longitudinal_speed * longitudinal_speed);
		if (horizontal_speed == 0 && vertical_speed == 0) return 0; // Neutral FPA
		if (horizontal_speed == 0 && vertical_speed < 0) return -90; // Straight down
		if (horizontal_speed == 0 && vertical_speed > 0) return 90; // Straight up
		return degrees(atan(vertical_speed / horizontal_speed));
	}
	double VFPARate()
	{
		return vfpa_rate;
	}
	double Vmo() { return vmo; }
	
	void Update(const double t, const double dt)
	{
		// Pre-update (for derived values)
		last_pitch = pitch;
		last_vfpa = VFPA();

		// Update
		aoa = FetchSimVar("INCIDENCE ALPHA", "Degrees", 0, 0);
		autopilot = FetchSimVar("AUTOPILOT MASTER", "Bool", 0, FALSE) == TRUE;
		flaps = static_cast<int>(FetchSimVar("FLAPS HANDLE INDEX", "Number", 0, 0));
		gforce = FetchSimVar("G FORCE", "GForce", 0, 0);
		ias = FetchSimVar("AIRSPEED INDICATED", "Knots", 0, 0);
		lateral_speed = FetchSimVar("VELOCITY WORLD Z", "Feet per second", 0, 0);
		longitudinal_speed = FetchSimVar("VELOCITY WORLD X", "Feet per second", 0, 0);
		mach = FetchSimVar("AIRSPEED MACH", "Mach", 0, 0);
		mmo = FetchSimVar("BARBER POLE MACH", "Mach", 0, DBL_MAX); // TODO: Get this data from the FCOM instead of the SimVar
		on_ground = FetchSimVar("SIM ON GROUND", "Bool", 0, FALSE) == TRUE;
		pitch = -FetchSimVar("PLANE PITCH DEGREES", "Degrees", 0, 0);
		radio_height = FetchSimVar("RADIO HEIGHT", "Feet", 0, 0);
		roll = -FetchSimVar("PLANE BANK DEGREES", "Degrees", 0, 0);
		vertical_speed = FetchSimVar("VELOCITY WORLD Y", "Feet per second", 0, 0);
		vmo = FetchSimVar("AIRSPEED BARBER POLE", "Knots", 0, DBL_MAX); // TODO: Get this data from the FCOM instead of the SimVar

		// Derived values
		pitch_rate = (pitch - last_pitch) / dt;
		vfpa_rate = (VFPA() - last_vfpa) / dt;
	}
};

AircraftData aircraft_data;