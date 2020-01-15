#include <array>
#include "RigidBodySimulationTypes.h"
#include "util/util.h"
#include "DrawingUtilitiesClass.h"
#include "SimpleMath.h"

MuTime g_debugTimer = MuTime().update();

Box::Box()
{
	calculateInertiaTensor();
}

Box::Box(Vec3 position)
	: m_position(position)
{
	calculateInertiaTensor();
}

Box::Box(Vec3 position, Vec3 size)
	: Box(position)
{
	m_extents = size;
}

Box::Box(Vec3 position, Vec3 size, double mass)
	: Box(position, size)
{
	m_mass = mass;
}

Mat4 Box::getRotScaleMatrix() const
{
	Mat4 scale;
	scale.initScaling(m_extents.x, m_extents.y, m_extents.z);
	return scale * m_rotation.getRotMat();
}

Mat4 Box::asMatrix() const
{
	Mat4 position;
	position.initTranslation(m_position.x, m_position.y, m_position.z);
	return getRotScaleMatrix() * position;
}

std::vector<Vec3> Box::getVerticesWorldSpace() const
{
	float edgelength = 1.0;
	auto points = std::vector<Vec3>{};
	auto oneAndZero = std::vector<float>{-0.5f * edgelength,0.5f * edgelength};
	auto transform = asMatrix();
	for (auto& x: oneAndZero)
	{
		for (auto& y : oneAndZero)
		{
			for (auto& z : oneAndZero)
			{
				points.push_back(transform.transformVector(Vec3(x, y, z)));
			}
		}
	}
	return points;
}

void Box::calculateInertiaTensor()
{
	// formula for moment of inertia of cuboids from https://en.wikipedia.org/wiki/List_of_moments_of_inertia
	
	const double oneTwelvthMass = m_mass / 12.0;
	m_inertiaTensorInverse.value[0][0] = 1.0 / (oneTwelvthMass * (sqr(m_extents.y) + sqr(m_extents.z)));
	m_inertiaTensorInverse.value[1][1] = 1.0 / (oneTwelvthMass * (sqr(m_extents.x) + sqr(m_extents.z)));
	m_inertiaTensorInverse.value[2][2] = 1.0 / (oneTwelvthMass * (sqr(m_extents.x) + sqr(m_extents.y)));
}

const Vec3 Box::getPointVelocityWorldSpace(const Vec3 relativePointWorldSpace) const
{
	
	return m_velocity + cross(getAngularVelocity(), relativePointWorldSpace);
}

const Mat4d Box::getInertiaTensorInverseWorldSpace() const
{
	return m_rotation.getRotMat() * m_inertiaTensorInverse * m_rotation.getRotMat().inverse();
}

const bool Box::haveSphereBVIntersection(const Box & other) const
{
	const auto& B = other;
	float maxDimensionA = max(m_extents.x, max(m_extents.y, m_extents.z));
	float maxDimensionB = max(B.m_extents.x, max(B.m_extents.y, B.m_extents.z));
	if (m_position.squaredDistanceTo(other.m_position) > (square(maxDimensionA) + square(maxDimensionB)))
	{
		return false;
	}
	return true;
}

const bool Box::isPointInsideBoundingSphere(const Vec3 & point) const
{
	float maxDimensionA = max(m_extents.x, max(m_extents.y, m_extents.z));
	if (m_position.squaredDistanceTo(point) > square(maxDimensionA))
	{
		return false;
	}
	return true;
}

const bool Box::hasCollisionWithPoint(const Vec3 point, Vec3 & out_collisionPoint) const
{
	if (!isPointInsideBoundingSphere(point))
		return false;
	// Transform point to BB's space
	const auto relativePoint = point - this->m_position;
	auto pointBBspace = this->m_rotation.getRotMat().inverse().operator*(relativePoint);
	pointBBspace = pointBBspace*2;
	// Detect if point is within extent
	const auto isInRange = [](const float x, const float low, const float high) { return x >= low && x <= high; };
	bool isInside = isInRange(pointBBspace.x, -this->m_extents.x, this->m_extents.x)
		&& isInRange(pointBBspace.y, -this->m_extents.y, this->m_extents.y)
		&& isInRange(pointBBspace.z, -this->m_extents.z, this->m_extents.z);
	if (!isInside)
		return false;
	// Otherwise, project point to planes and return the point on closest plane
	// Algorithm: for each dimension, calculate distance to the closest plane
	// and take the dimension with lowest distance, and add its distance to point
	auto distancesToClosestPlane = this->m_extents - (pointBBspace.getAbsolutes());
	auto getIndexOfMinimalDimension = [](const Vec3 p)
	{
		if (p.x < p.y)
		{
			if (p.x < p.z)
				return 0;
			else
				return 2;
		}
		else {

			if (p.y < p.z)
				return 1;
			else
				return 2;
		}
	};
	const auto index = getIndexOfMinimalDimension(distancesToClosestPlane);
	const auto sign = (pointBBspace[index] > 0)?1:-1;
	// Store closest point on plane according to orthogonal projection
	out_collisionPoint = pointBBspace;
	out_collisionPoint[index] += sign * distancesToClosestPlane[index];
	// Transform back to world coords
	out_collisionPoint = this->m_rotation.getRotMat() * (out_collisionPoint*0.5);
	out_collisionPoint += this->m_position;
	return true;
}

const Vec3 Box::getAngularVelocity() const
{
	return getInertiaTensorInverseWorldSpace()*m_angularMomentum;
}

Vec3 PlanarConstraint::projectOntoPlane(Vec3 point) const
{
	const Vec3 relativeVector = point - position;
	return position + projectDirectionOntoPlane(relativeVector);
}

Vec3 PlanarConstraint::projectDirectionOntoPlane(Vec3 direction) const
{
	return direction - dot(direction, normal) * normal;
}

double PlanarConstraint::distanceToPlane(Vec3 point) const
{
	const Vec3 relativeVector = point - position;
	return dot(relativeVector, normal);
}

void Ray::Draw(DrawingUtilitiesClass* DUC) const
{
	DUC->drawLine(start, colorStart, start + direction * length, colorEnd);
}

void Line::Draw(DrawingUtilitiesClass* DUC) const
{
	DUC->drawLine(start, colorStart, end, colorEnd);
}