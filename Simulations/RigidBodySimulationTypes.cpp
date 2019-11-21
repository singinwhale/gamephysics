#include "RigidBodySimulationTypes.h"
#include "util/util.h"
#include "SimpleMath.h"

Box::Box()
{
	this->initialize();
}

Box::Box(Vec3 position)
	: m_position(position)
{
	this->initialize();
}

Box::Box(Vec3 position, Vec3 size)
	: Box(position)
{
	m_extents = size;
}

Box::Box(Vec3 position, Vec3 size, float mass)
	: Box(position, size)
{
	m_mass = mass;
}

Mat4 Box::getRotScaleMatrix() const
{
	Mat4 rotation = m_rotation.getRotMat();
	Mat4 scale;
	scale.initScaling(m_extents.x, m_extents.y, m_extents.z);
	return scale * rotation;
}

Mat4 Box::asMatrix() const
{
	Mat4 rotationScale = getRotScaleMatrix();
	Mat4 position;
	position.initTranslation(m_position.x, m_position.y, m_position.z);
	return rotationScale * position;
}

std::vector<Vec3> Box::getVertices() const
{
	auto points = std::vector<Vec3>{};
	auto oneAndZero = std::vector<float>{-1.0f,1.0f};
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

void Box::initialize()
{
	VERBOSE(std::cout << "Box::initialize()" << std::endl)
	const double oneTwelvth = 1.0 / 12.0;
	m_inertiaTensorInverse.value[0][0] = oneTwelvth * m_mass * (sqr(m_extents.y) + sqr(m_extents.z));
	m_inertiaTensorInverse.value[1][1] = oneTwelvth * m_mass * (sqr(m_extents.x) + sqr(m_extents.z));
	m_inertiaTensorInverse.value[2][2] = oneTwelvth * m_mass * (sqr(m_extents.x) + sqr(m_extents.y));
	
}

const Vec3 Box::getRelativePositionFromWorld(const Vec3 worldPosition) const
{
	return this->asMatrix().inverse().transformVector(worldPosition);
}

const Vec3 Box::getRelativeDirectionFromWorld(const Vec3 worldPosition) const
{
	return this->asMatrix().inverse().transformVector(worldPosition);
}

const Vec3 Box::getPointVelocity(const Vec3 relativePoint) const
{
	return m_velocity + cross(m_angularMomentum, relativePoint);
}

const Mat4d Box::getInertiaTensorInverseWorldSpace() const
{
	return m_rotation.getRotMat().inverse() * m_inertiaTensorInverse * m_rotation.getRotMat();
}

Vec3 PlanarConstraint::projectOntoPlane(Vec3 point) const
{
	const Vec3 relativeVector = point - position;
	return position + relativeVector - dot(relativeVector, normal) * normal;
}

double PlanarConstraint::distanceToPlane(Vec3 point) const
{
	const Vec3 relativeVector = point - position;
	return dot(relativeVector, normal);
}
