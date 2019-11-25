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
	
	return m_velocity + cross(m_angularMomentum, relativePointWorldSpace);
}

const Mat4d Box::getInertiaTensorInverseWorldSpace() const
{
	return m_rotation.getRotMat() * m_inertiaTensorInverse * m_rotation.getRotMat().inverse();
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