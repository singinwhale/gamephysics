#include "RigidBodySimulationTypes.h"


Box::Box()
{
}

Box::Box(Vec3 position)
	: m_position(position)
{
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

Mat4 Box::asMatrix() const
{
	Mat4 rotation = m_rotation.getRotMat();
	Mat4 position;
	position.initTranslation(m_position.x, m_position.y, m_position.z);
	Mat4 scale;
	scale.initScaling(m_extents.x, m_extents.y, m_extents.z);
	return scale * rotation * position;
}
