
#include "RigidBodySystemSimulator.h"
#include "RigidBodySimulationTypes.h"
#include "RigidBodySystem.h"

#define REINTERPRET_RBSS(VarName, rigidBodySystemSimulatorPtr) RigidBodySystemSimulator* VarName = reinterpret_cast<RigidBodySystemSimulator*>(rigidBodySystemSimulatorPtr)

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = TESTCASEUSEDTORUNTEST;
	reset();
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo 1,Demo 2,Demo 3";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	
	TwAddVarRW(DUC->g_pTweakBar, "Box Size", TW_TYPE_DIR3F, &m_defaultBoxSize[0], "");
	TwAddVarRW(DUC->g_pTweakBar, "Friction", TW_TYPE_DOUBLE, &m_params.friction, "");
	TwAddVarRW(DUC->g_pTweakBar, "Angular Damping", TW_TYPE_DOUBLE, &m_params.angularDamping, "");
	TwAddVarRW(DUC->g_pTweakBar, "Linear Damping", TW_TYPE_DOUBLE, &m_params.linearDamping, "");
	TwAddVarRW(DUC->g_pTweakBar, "DebugLifetime", TW_TYPE_UINT32, &m_debugLifetime, "");
	TwAddVarRW(DUC->g_pTweakBar, "Roomscale", TW_TYPE_DOUBLE, &m_roomScale, "");
	TwAddVarRW(DUC->g_pTweakBar, "Default Box Mass", TW_TYPE_DOUBLE, &m_defaultMass, "");
	TwAddVarRW(DUC->g_pTweakBar, "Debug", TW_TYPE_BOOL8, &m_params.showDebugInfo, "");
	TwAddButton(DUC->g_pTweakBar, "Add Box", [](void* data)
	{
		REINTERPRET_RBSS(rbss, data);
		Vec3 size = Vec3(rbss->m_defaultBoxSize[0], rbss->m_defaultBoxSize[1], rbss->m_defaultBoxSize[2]);
		//rbss->addRigidBody(Vec3::ZERO, size, size.x*size.y*size.z);
		std::size_t indexToBox = rbss->addRigidBody(Vec3::ZERO, size * (Vec3::randomDir()/2.0 + Vec3(0.5)), rbss->m_defaultMass);
		rbss->setOrientationOf(indexToBox, Quat::getRandom());
	}, this, "");
	TwAddButton(DUC->g_pTweakBar, "Toggle gravity", [](void* data)
	{
		REINTERPRET_RBSS(rbss, data);
		bool hasGravity = (rbss->m_params.constantAcceleration.squaredDistanceTo(Vec3(0.0)) > 0.0);
		rbss->setConstantForce(hasGravity ? Vec3(0.0) : Vec3(0.0, -9.81, 0.0));
	}, this, "");

	TwAddButton(DUC->g_pTweakBar, "Add left force to last object", [](void* data)
	{
		REINTERPRET_RBSS(rbss, data);
		const auto count = rbss->getNumberOfRigidBodies();
		rbss->applyForceOnBody(count - 1, Vec3(1.0, 0.0, 0.0), Vec3(0.0, 30.0, 0.0));
	}, this, "");

	TwAddButton(DUC->g_pTweakBar, "Add right force to last object", [](void* data)
	{
		REINTERPRET_RBSS(rbss, data);
		const auto count = rbss->getNumberOfRigidBodies();
		rbss->applyForceOnBody(count - 1, Vec3(-1.0, 0.0, 0.0), Vec3(0.0, 30.0, 0.0));
	}, this, "");

	TwAddButton(DUC->g_pTweakBar, "Demo 1 print", [](void* data)
	{
		REINTERPRET_RBSS(rbss, data);
		RigidBodySystem demo;
		demo.m_angularDamping = 0;
		demo.m_linearDamping = 0;
		demo.m_constraints.clear();
		demo.m_constantAcceleration = Vec3(0.0);
		demo.m_params.angularDamping = 0;
		demo.m_params.constantAcceleration = 0;
		demo.m_params.friction = 0;
		demo.m_params.linearDamping = 0;

		demo.m_rigid_bodies.push_back(Box(Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.6, 0.5), 2.0));
		demo.m_rigid_bodies[0].m_rotation = Quat(Vec3(0, 0, 1.0), XMConvertToRadians(90.0));
		demo.m_rigid_bodies[0].m_forceApplications.push_back(std::make_pair(Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0)));
		demo.initialize();
		
		demo.tick(2.0);	
		auto& point = demo.m_rigid_bodies[0];
		std::cout << "InertiaTensor Inverse " << point.m_inertiaTensorInverse << std::endl;
		auto velocityPoint = point.getPointVelocityWorldSpace(Vec3(0.3, 0.5, 0.25));
		std::cout << "Demo1 after h=2: Velocity at (0.3, 0.5, 0.25) " << velocityPoint << std::endl;
		std::cout << "Angular velocity: " << point.getAngularVelocity() << std::endl;
		std::cout << "Linear velocity" << point.m_velocity << std::endl;
	}, this, "");
}

void RigidBodySystemSimulator::addWallAndFloorConstraints()
{
	double scale = m_roomScale;
	addConstraint(Vec3(1,0,0)*scale, Vec3(-1,0,0));
	addConstraint(Vec3(-1,0,0)*scale, Vec3(1,0,0));
	addConstraint(Vec3(0,1,0)*scale, Vec3(0,-1,0));
	addConstraint(Vec3(0,-1,0)*scale, Vec3(0,1,0));
	addConstraint(Vec3(0,0,1)*scale, Vec3(0,0,-1));
	addConstraint(Vec3(0,0,-1)*scale, Vec3(0,0,1));
}

void RigidBodySystemSimulator::reset()
{
	m_pRigidBodySystem = std::make_unique<RigidBodySystem>();
	m_pRigidBodySystem->m_params = m_params;

	Vec3 size = Vec3(0.1);
	switch(m_iTestCase)
	{
	case 0:
	{		
		const auto index = this->getNumberOfRigidBodies();
		this->addRigidBody(Vec3(0.3, 0.0, 0.0), size, m_defaultMass);
		this->setVelocityOf(index, Vec3(-1.0, 0.0, 0.0));
		this->setOrientationOf(0, Quat(Vec3(1, 0, 0), M_PI / 2.0));
		
		this->addRigidBody(Vec3(0.0, 0.0, 0.0), size, m_defaultMass);

		addWallAndFloorConstraints();
	}
		break;
	case 1:

		this->addRigidBody(Vec3(0.35, 0.35, 0.0), size, m_defaultMass);
		addConstraint(Vec3(0, -0.46, 0), getNormalized(Vec3(-1, 5, 0)));
		addWallAndFloorConstraints();
		break;
	case 2:
	default:
		m_params.angularDamping = 0;
		m_params.linearDamping = 0;
		m_params.constantAcceleration = Vec3::ZERO;
		m_params.friction = 0;

		addRigidBody(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);

		addRigidBody(Vec3(0.0f, 0.2f, 0.0f), Vec3(0.4f, 0.2f, 0.2f), 100.0);
		setOrientationOf(1, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI)*0.25f));
		setVelocityOf(1, Vec3(0.0f, -0.1f, 0.05f));
		break;
	}
}

template<class DebugType>
void removeOldDebugObjects(std::vector<DebugType>& InContainer, uint64_t lifetime)
{

	for (std::size_t i = 0; i < InContainer.size(); ++i)
	{
		DebugType& Straight = InContainer[i];
		if (Straight.timestamp + lifetime < g_debugTimer.time)
		{
			continue;
		}
		InContainer.erase(InContainer.begin(), InContainer.begin() + i);
		return;
	}
}

void RigidBodySystemSimulator::drawDebugLines() const
{
	removeOldDebugObjects(m_pRigidBodySystem->m_debugLines, uint64_t(m_debugLifetime));
	for (std::size_t i = 0; i < m_pRigidBodySystem->m_debugLines.size(); ++i)
	{
		m_pRigidBodySystem->m_debugLines[i].Draw(DUC);
	}
}

void RigidBodySystemSimulator::drawDebugRays() const
{
	removeOldDebugObjects(m_pRigidBodySystem->m_debugRays, uint64_t(m_debugLifetime));
	for (std::size_t i = 0; i < m_pRigidBodySystem->m_debugRays.size(); ++i)
	{		
		m_pRigidBodySystem->m_debugRays[i].Draw(DUC);
	}
}

void RigidBodySystemSimulator::drawConstraints() const
{
	for(const PlanarConstraint& constraint: m_pRigidBodySystem->m_constraints)
	{
		DUC->drawLine(constraint.position, Vec3(1, 0, 0), constraint.position + constraint.normal * 0.05, Vec3(0, 1, 0));
	}
}

void RigidBodySystemSimulator::drawBoxes()
{
	for(const Box& box : m_pRigidBodySystem->m_rigid_bodies)
	{
		DUC->drawRigidBody(box.asMatrix());
	}
}

void RigidBodySystemSimulator::drawAngularMomentum()
{
	for (const Box& box : m_pRigidBodySystem->m_rigid_bodies)
	{
		DUC->drawLine(box.m_position, Vec3(0, 0, 1), box.m_position + box.m_angularMomentum * 0.5, Vec3(0, 1, 1));
	}
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3::ZERO, Vec3(1, 1, 1), 0.1, Vec3(.9, .9, .9));
	drawBoxes();
	
	DUC->beginLine();
	drawConstraints();
	drawDebugLines();
	drawDebugRays();
	if(m_params.showDebugInfo)
		drawAngularMomentum();
	DUC->endLine();
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	reset();
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	//TODO
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	g_debugTimer.update();
	m_pRigidBodySystem->m_params = m_params;
	m_pRigidBodySystem->tick(timeStep);
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_oldtrackmouse = m_oldtrackmouse;
	m_trackmouse = { x,y };

	// Find closest rigid body within intersection
	auto rayDirection = this->screenToRay(x, y);
	auto unitDirection = rayDirection / norm(rayDirection);
	auto rayStart = this->getCameraPosition();

	bool hasIntersection = false;
	Box* intersectedBody = nullptr;
	Vec3 intersectedPosition;
	float minLambda = 1e100;
	for (auto& body : this->m_pRigidBodySystem->m_rigid_bodies)
	{
		// Compute orthogonal projection of point to line
		auto intersectionPoint = this->pointToRayProjection(rayStart,rayDirection,body.m_position);
		float lambda = norm(intersectionPoint - rayStart);
		if (minLambda > lambda)
		{
			auto diameter = max(body.m_extents.x, max(body.m_extents.y, body.m_extents.z));
			if (diameter >= sqrt(intersectionPoint.squaredDistanceTo(body.m_position)))
			{
				minLambda = lambda;
				intersectedBody = &body;
				intersectedPosition = intersectionPoint;
				hasIntersection = true;
			}
		}
		
	}
	// Add force to rigit body
	if (hasIntersection)
	{
		auto localForcePositions = intersectedPosition-intersectedBody->m_position;
		std::cout << "Applying force at point: " << localForcePositions << std::endl;

		intersectedBody->m_forceApplications.push_back(std::make_pair(localForcePositions, -unitDirection*1000));
		//intersectedBody->m_forceApplications.push_back(std::make_pair(localForcePositions, unitDirection*100));
	}

}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_mouse = { x,y };
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return m_pRigidBodySystem->m_rigid_bodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return m_pRigidBodySystem->m_rigid_bodies[i].m_position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_pRigidBodySystem->m_rigid_bodies[i].m_velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_pRigidBodySystem->m_rigid_bodies[i].m_angularMomentum;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_pRigidBodySystem->m_rigid_bodies[i].m_forceApplications.push_back(std::make_pair(loc, force));
}

std::size_t RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, double mass)
{
	m_pRigidBodySystem->m_rigid_bodies.push_back(Box(position, size, mass));
	return m_pRigidBodySystem->m_rigid_bodies.size() - 1;
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_pRigidBodySystem->m_rigid_bodies[i].m_rotation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_pRigidBodySystem->m_rigid_bodies[i].m_velocity = velocity;
}

void RigidBodySystemSimulator::setConstantForce(Vec3 force)
{
	m_params.constantAcceleration = force;
}

Vec3 RigidBodySystemSimulator::getConstantForce() const
{
	return m_pRigidBodySystem->m_constantAcceleration;
}

void RigidBodySystemSimulator::addConstraint(Vec3 position, Vec3 normal)
{
	m_pRigidBodySystem->m_constraints.push_back({ position,getNormalized(normal)});
}

Vec3 RigidBodySystemSimulator::screenToRay(const float px, const float py)
{
	auto screenSize = this->getScreenSize();
	DirectX::XMMATRIX projectionMatrix, viewMatrix, inverseViewMatrix;
	projectionMatrix = this->DUC->g_camera.GetProjMatrix();
	viewMatrix = this->DUC->g_camera.GetViewMatrix();

	auto pixelVector = DirectX::XMVectorSet(px, py, 0.0, 1.0);
	auto resultVector = DirectX::XMVector3Unproject(pixelVector, 0.0, 0.0, screenSize.x, screenSize.y, 0.0, 1.0, projectionMatrix, viewMatrix, DirectX::XMMatrixIdentity());

	return Vec3(DirectX::XMVectorGetX(resultVector), DirectX::XMVectorGetY(resultVector), DirectX::XMVectorGetZ(resultVector));
}

Vec3 RigidBodySystemSimulator::getScreenSize()
{
	D3D11_VIEWPORT viewport;
	UINT viewportsCount = 1;
	this->DUC->g_pd3dImmediateContext->RSGetViewports(&viewportsCount, &viewport);
	return Vec3(viewport.Width, viewport.Height, 1.0);
}

Vec3 RigidBodySystemSimulator::getCameraPosition()
{
	DirectX::XMMATRIX projectionMatrix, viewMatrix, inverseViewMatrix;
	viewMatrix = this->DUC->g_camera.GetViewMatrix();
	inverseViewMatrix = DirectX::XMMatrixInverse(nullptr, viewMatrix);
	auto cameraOrigin = DirectX::XMVectorSet(0, 0, 0, 1.0);
	auto cameraPositionVector = DirectX::XMVector4Transform(cameraOrigin, inverseViewMatrix);
	auto& cpv = cameraPositionVector;
	return Vec3(DirectX::XMVectorGetX(cpv), ::XMVectorGetY(cpv), ::XMVectorGetZ(cpv));
}

Vec3 RigidBodySystemSimulator::pointToRayProjection(Vec3 start, Vec3 end, Vec3 point)
{
	auto lineVec = end - start;
	auto linePoint = point - start;
	auto l = norm(linePoint);
	auto nLine = lineVec / norm(lineVec);
	auto nP = linePoint / norm(linePoint);
	auto angle = dot(nLine, nP);
	return l * angle*nLine + start;
}
