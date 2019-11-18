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

void Box::initialize()
{
	auto points = std::vector<Vec3>{};
	auto oneAndZero = std::vector<float>{0.0f,1.0f};
	for (auto& x: oneAndZero)
	{
		for (auto& y : oneAndZero)
		{
			for (auto& z : oneAndZero)
			{
				points.push_back(Vec3(x, y, z));
			}
		}
	}

	Mat4d covariance;
	float mass = this->m_mass/points.size();
	//TODO: calculate center of I0 (initertia vector)
	for (auto& point: points)
	{
		Mat4d t,tTranspose;
		t.value[0][0] = point.x;
		t.value[0][1] = point.y;
		t.value[0][2] = point.z;

		tTranspose = t;
		t.transpose();
		t = tTranspose*t;
		covariance += t * mass;
	}
	double trace = covariance.value[1][1] + covariance.value[2][2] + covariance.value[3][3];
	Mat4d traceMat;
	traceMat.value[1][1] = trace;
	traceMat.value[2][2] = trace;
	traceMat.value[3][3] = trace;
	m_inertiaTensorInverse = traceMat - covariance;

}
