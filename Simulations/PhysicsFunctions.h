#pragma once
#include "util/matrixbase.h"

using namespace GamePhysics;

namespace Physics
{

	inline Vec3 CalculateSpringForceBetweenPoints(Vec3 pointA, Vec3 pointB, float springRestlength, float springConstant)
	{
		return -springConstant * (pointA - pointB) / springRestlength;
	}

	inline Vec3 CalculateDampingForce(Vec3 velocity, float dampingFactor)
	{
		return -velocity * dampingFactor;
	}

}