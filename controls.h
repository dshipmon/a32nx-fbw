#pragma once
#include "common.h"
#include "aircraft_data.h"
#include "input.h"
#include "roll.h"
#include "pitch.h"

class ControlSurfaces
{
private:
	struct CONTROL_SURFACES_DATA
	{
		double elevator = 0; // -1 is full down, and +1 is full up
		double ailerons = 0; // -1 is full left, and +1 is full right
		double rudder = 0; // -1 is full left, and +1 is full right
	} control_surfaces;

	enum DEFINITION_ID
	{
		CONTROL_SURFACES_DEFINITION
	};

	RollController roll_controller = RollController();
	PitchController pitch_controller = PitchController();
public:
	void Init()
	{
		SimConnect_AddToDataDefinition(hSimConnect, CONTROL_SURFACES_DEFINITION, "ELEVATOR POSITION", "Position");
		SimConnect_AddToDataDefinition(hSimConnect, CONTROL_SURFACES_DEFINITION, "AILERON POSITION", "Position");
		SimConnect_AddToDataDefinition(hSimConnect, CONTROL_SURFACES_DEFINITION, "RUDDER POSITION", "Position");
	}
	void Update(const double t, const double dt)
	{
		if (aircraft_data.Autopilot())
		{
			// Allow the user's raw flight control inputs to go through if the autopilot is on
			control_surfaces.elevator = input_capture.RawYokeY();
			control_surfaces.ailerons = input_capture.RawYokeX();
			control_surfaces.rudder = input_capture.RawRudder();
		}
		else
		{
			// We are controlling the plane through FBW
			control_surfaces.ailerons = roll_controller.Calculate(control_surfaces.ailerons, t, dt);
			control_surfaces.elevator = pitch_controller.Calculate(control_surfaces.elevator, t, dt);
			control_surfaces.rudder = input_capture.RawRudder(); // TODO: Create yaw FBW
		}
		SimConnect_SetDataOnSimObject(hSimConnect, CONTROL_SURFACES_DEFINITION, SIMCONNECT_OBJECT_ID_USER, 0, 0, sizeof(control_surfaces), &control_surfaces);
	}
};

ControlSurfaces control_surfaces;