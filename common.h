#pragma once

#include <MSFS/MSFS.h>
#include <MSFS/MSFS_Render.h>
#include <MSFS/Legacy/gauges.h>
#include <SimConnect.h>
#include <cmath>

HANDLE hSimConnect = 0;

constexpr double clamp(const double value, const double min, const double max)
{
	return (value < min) ? min : (value > max) ? max : value;
}

constexpr double sign(const double value)
{
	return (value > 0) ? 1.0 : -1.0;
}

constexpr double radians(const double value)
{
	return value * (M_PI / 180);
}
constexpr double degrees(const double value)
{
	return value * (180 / M_PI);
}

constexpr double linear_decay_coefficient(const double position, const double start, const double end)
{
	// This function provides a coefficient such that the effectiveness is maximal (1.0) before the start,
	// reduces linearly up to the end, and after which is no longer effective (0.0).
	//
	// Effect in the positive direction:
	// <--- maximum effectiveness (1.0) ---> start <--- linear change to effectiveness ---> end <--- no effectiveness (0.0) --->
	// Effect in the negative direction:
	// <--- no effectiveness (0.0) ---> end <--- linear change to effectiveness ---> start <--- maximum effectiveness (1.0) --->

	if ((start < end && position <= start) || (start >= end && position >= start))
	{
		return 1.0;
	}
	else if ((start < end && position >= end) || (start >= end && position <= end))
	{
		return 0.0;
	}
	else
	{
		return 1.0 - ((position - start) / (end - start));
	}
}

// Scales a coefficient 0.0->1.0 from min->max
constexpr double linear_range(const double coefficient, const double min, const double max)
{
	return (max - min) * coefficient + min;
}
