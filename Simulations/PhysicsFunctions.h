#pragma once
#include "util/matrixbase.h"

using namespace GamePhysics;

namespace Physics
{

	inline Vec3 CalculateSpringForceBetweenPoints(Vec3 pointA, Vec3 pointB, double springRestlength, double springConstant)
	{
		const Vec3 springVector = pointB - pointA;
		double springLength = norm(springVector);
		// Hack: if points are infinitely close together, then force is zero (to prevent infty)
		if (springLength < std::numeric_limits<double>::epsilon())
		{
			return Vec3();
		}
		const double forceMagnitude = -springConstant * (springLength - springRestlength);
		return (springVector / springLength) * forceMagnitude;
	}

	inline Vec3 CalculateDampingForce(Vec3 velocity, double dampingFactor)
	{
		return -velocity * dampingFactor;
	}

	inline Vec3 eulerIntegration(Vec3 value, Vec3 slope, double deltaSeconds)
	{
		return value + slope * deltaSeconds;
	}

}