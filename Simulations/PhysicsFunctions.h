#pragma once
#include "util/matrixbase.h"

using namespace GamePhysics;

namespace Physics
{

	inline Vec3 CalculateSpringForceBetweenPoints(Vec3 pointA, Vec3 pointB, double springRestlength, double springConstant)
	{
		const Vec3 springVector = pointB - pointA;
		const double springLength = norm(springVector);
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