#include "pitch_control_mode.h"
#include "input.h"
#include "protections.h"
#include "controls.h"

#define ENABLE_FBW_SYSTEM TRUE

extern "C"
{
	MSFS_CALLBACK bool FBW_gauge_callback(FsContext ctx, const int service_id, void* pData)
	{
		auto ret = true;
		switch (service_id)
		{
		case PANEL_SERVICE_PRE_INSTALL:
		{
			ret &= SUCCEEDED(SimConnect_Open(&hSimConnect, "A32NX_FBW", nullptr, 0, 0, 0));
		}
		break;
		case PANEL_SERVICE_POST_INSTALL:
		{
			if (ENABLE_FBW_SYSTEM)
			{
				input_capture.Init();
				control_surfaces.Init();
			}
		}
		break;
		case PANEL_SERVICE_PRE_DRAW:
		{
			// Sent before the gauge is drawn. The pData parameter points to a sGaugeDrawData structure:
			// - The t member gives the absolute simulation time.
			// - The dt member gives the time elapsed since last frame.
			auto* p_draw_data = static_cast<sGaugeDrawData*>(pData);
			const auto t = p_draw_data->t;
			const auto dt = p_draw_data->dt;
			if (ENABLE_FBW_SYSTEM)
			{
				aircraft_data.Update(t, dt);
				pitch_control_mode.Update(t, dt);
				normal_law_protections.Update(t, dt);
				input_capture.Update(t, dt);
				control_surfaces.Update(t, dt); // Calls the FBW logic internally
			}
		}
		break;
		case PANEL_SERVICE_PRE_KILL:
		{
			ret &= SUCCEEDED(SimConnect_Close(hSimConnect));
		}
		break;
		default: break;
		}
		return ret;
	}
}
