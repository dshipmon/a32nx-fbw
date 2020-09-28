#pragma once
#include "common.h"


enum EVENT_ID
{
	// Elevator Group
	ELEVATOR_SET_EVENT, // AXIS_ELEVATOR_SET
	// Aileron Group
	AILERONS_SET_EVENT, // AXIS_AILERONS_SET
	CENTER_AILERONS_RUDDER_EVENT, // CENTER_AILER_RUDDER
	// Rudder group
	RUDDER_SET_EVENT, // AXIS_RUDDER_SET
	RUDDER_CENTER_EVENT, // RUDDER_CENTER
};

void CALLBACK OnInputCaptureEvent(SIMCONNECT_RECV* pData, DWORD cbData, void* pContext);

class InputCapture
{
private:
	double yoke_y; // -1 is full down, and +1 is full up
	double yoke_x; // -1 is full left, and +1 is full right
	double rudder; // -1 is full left, and +1 is full right

	enum GROUP_ID
	{
		ELEVATOR_GROUP,
		AILERON_GROUP,
		RUDDER_GROUP
	};
	
	double PositionWithNullZone(const double position, const double null_zone_percent)
	{
		return sign(position) * linear_decay_coefficient(position, sign(position), sign(position) * null_zone_percent / 2.0);
	}
public:	
	double RawYokeY() { return yoke_y; }
	double RawYokeX() { return yoke_x; }
	double RawRudder() { return rudder; }
	
	double YokeY() { return PositionWithNullZone(yoke_y, 0.10); }
	double YokeX() { return PositionWithNullZone(yoke_x, 0.10); }
	double Rudder() { return rudder; }

	void SetYokeY(const double value) { yoke_y = value; }
	void SetYokeX(const double value) { yoke_x = value; }
	void SetRudder(const double value) { rudder = value; }
	
	void Init()
	{
		// Register input capture
		// Client events reference: http://www.prepar3d.com/SDKv3/LearningCenter/utilities/variables/event_ids.html
		// Elevator group
		SimConnect_MapClientEventToSimEvent(hSimConnect, ELEVATOR_SET_EVENT, "AXIS_ELEVATOR_SET");
		SimConnect_AddClientEventToNotificationGroup(hSimConnect, ELEVATOR_GROUP, ELEVATOR_SET_EVENT, TRUE);

		// Aileron group
		SimConnect_MapClientEventToSimEvent(hSimConnect, AILERONS_SET_EVENT, "AXIS_AILERONS_SET");
		SimConnect_MapClientEventToSimEvent(hSimConnect, CENTER_AILERONS_RUDDER_EVENT, "CENTER_AILER_RUDDER");
		SimConnect_AddClientEventToNotificationGroup(hSimConnect, AILERON_GROUP, AILERONS_SET_EVENT, TRUE);
		SimConnect_AddClientEventToNotificationGroup(hSimConnect, AILERON_GROUP, CENTER_AILERONS_RUDDER_EVENT, TRUE);

		// Rudder group
		SimConnect_MapClientEventToSimEvent(hSimConnect, RUDDER_SET_EVENT, "AXIS_RUDDER_SET");
		SimConnect_MapClientEventToSimEvent(hSimConnect, RUDDER_CENTER_EVENT, "RUDDER_CENTER");
		SimConnect_AddClientEventToNotificationGroup(hSimConnect, RUDDER_GROUP, RUDDER_SET_EVENT, TRUE);
		SimConnect_AddClientEventToNotificationGroup(hSimConnect, RUDDER_GROUP, RUDDER_CENTER_EVENT, TRUE);

		// Set maskable notification priorities
		SimConnect_SetNotificationGroupPriority(hSimConnect, ELEVATOR_GROUP, SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE);
		SimConnect_SetNotificationGroupPriority(hSimConnect, AILERON_GROUP, SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE);
		SimConnect_SetNotificationGroupPriority(hSimConnect, RUDDER_GROUP, SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE);
	}
	void Update(const double t, const double dt)
	{
		SimConnect_CallDispatch(hSimConnect, OnInputCaptureEvent, nullptr);
	}
	void Destroy()
	{
		// TODO: Unregister all the input capture?
	}
};

InputCapture input_capture;

void CALLBACK OnInputCaptureEvent(SIMCONNECT_RECV* pData, DWORD cbData, void* pContext)
{
	if (pData->dwID == SIMCONNECT_RECV_ID_EVENT)
	{
		auto* evt = static_cast<SIMCONNECT_RECV_EVENT*>(pData);
		switch (evt->uEventID)
		{
		case ELEVATOR_SET_EVENT:
			input_capture.SetYokeY(0 - (static_cast<long>(evt->dwData) / 16384.0)); // scale from [-16384,16384] to [-1,1] and reverse the sign
			break;
		case AILERONS_SET_EVENT:
			input_capture.SetYokeX(0 - (static_cast<long>(evt->dwData) / 16384.0)); // scale from [-16384,16384] to [-1,1] and reverse the sign
			break;
		case CENTER_AILERONS_RUDDER_EVENT:
			input_capture.SetYokeX(0);
			input_capture.SetRudder(0);
			break;
		case RUDDER_SET_EVENT:
			input_capture.SetRudder(0 - (static_cast<long>(evt->dwData) / 16384.0)); // scale from [-16384,16384] to [-1,1] and reverse the sign
			break;
		case RUDDER_CENTER_EVENT:
			input_capture.SetRudder(0);
			break;
		default: break;
		}
	}
}

