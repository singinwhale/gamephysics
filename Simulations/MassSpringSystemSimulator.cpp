#include "MassSpringSystemSimulator.h"
#include "util/FFmpeg.h"
#include "EulerParticleIntegrator.h"
#include "MidpointParticleIntegrator.h"

//Vec3::ZERO is not defined in this compile unit so we create our own definition
const vector3Dim<double>  vector3Dim<double>::ZERO = Vec3(0, 0, 0);
const vector3Dim<float>  vector3Dim<float>::ZERO = vector3Dim(0, 0, 0);

void setupScene(Roman::System& system)
{
	system.m_particles.clear();
	system.m_springs.clear();

	size_t ropeCount = 4;
	system.m_particles.push_back(Roman::Particle(Vec3(0.0,4.0,0.0), Vec3(), 1, true));
	for (size_t i = 0; i < ropeCount; i++)
	{
		system.m_particles.push_back(Roman::Particle(Vec3(0.0, 4.0-i*0.2, 0.0), Vec3(), 1, false));
		system.m_springs.push_back(Roman::Spring(i, i+1, 40, 0.01));
	}
	system.m_particles.back().position.x = 2.0;
	system.m_particles.back().position.z = 2.0;
	/*system.m_particles.push_back(Roman::Particle(Vec3(1.0, 2.0, 0.0), Vec3(), 1, false));
	system.m_particles.push_back(Roman::Particle(Vec3(0.0, 0.5, 0.0), Vec3(), 1, false));

	system.m_springs.push_back(Roman::Spring(0, 1, 4,2.0));
	system.m_springs.push_back(Roman::Spring(1, 2, 4, 2.0));
	*/
}

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = EULER;
	/*m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	m_iNumSpheres    = 100;
	m_fSphereSize    = 0.05f;*/
	m_fMass = 1.0;
	m_particleIntegrator = std::make_unique<EulerParticleIntegrator>();

	setupScene(m_romanSys);
	m_romanInt.setSystem(&m_romanSys);
	m_romanInt.setGlobalForce(Vec3(0.0, -9.81, 0.0));
}

#define STR(content) #content

const char * MassSpringSystemSimulator::getTestCasesStr(){

	return "EULER,LEAPFROG,MIDPOINT";
}

void MassSpringSystemSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

		std::cout << "Reset" << std::endl;
		setupScene(m_romanSys);
}

void TW_CALL callbackAddButton(void* simulator)
{
	auto sim = reinterpret_cast<MassSpringSystemSimulator*>(simulator);
	
	auto index = sim->getNumberOfMassPoints();
	if (index == 0)
	{
		index = 1;
	}
	index--;

	auto x1 = static_cast<float>(rand())/RAND_MAX;
	auto x2 = static_cast<float>(rand())/RAND_MAX;
	auto x3 = static_cast<float>(rand()) / RAND_MAX;


	sim->addMassPoint(Vec3(x1, x3, 0.0), Vec3(0.0, 0.0, 0.0),true);
	sim->addMassPoint(Vec3(x2, x3+0.2, 0.0), Vec3(0.0, 0.0, 0.0), true);
	//sim->addSpring(index, index+1, 0.5);
	std::cout << "Add random string " << x1 << " - " << x2 << std::endl;
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;

	TwAddButton(DUC->g_pTweakBar,"Add random point", &callbackAddButton, this, "");
	switch (m_iTestCase)
	{
	case 0:break;
	case 1:
		//TwAddVarRW(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT32, &m_iNumSpheres, "min=1");
		//TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.01");
		break;
	case 2:break;
	default:break;
	}

	{
		Roman::System sys;
		sys.m_particles.push_back(Roman::Particle(Vec3(), Vec3(-1.0,0,0), 10, false));
		sys.m_particles.push_back(Roman::Particle(Vec3(0.0,2.0,0.0), Vec3(1, 0, 0), 10, false));

		sys.m_springs.push_back(Roman::Spring(0, 1, 40, 1.0));

	
		Roman::Euler integrator;
		integrator.setSystem(&sys);
		for (size_t i = 0; i < 3; i++)
		{
			std::cout << "iteration: " << i << std::endl;
			sys.printOutParticles();
			integrator.doStep(0.1);
		}
	}
	{
		Roman::System sys;
		sys.m_particles.push_back(Roman::Particle(Vec3(), Vec3(-1.0, 0, 0), 10, false));
		sys.m_particles.push_back(Roman::Particle(Vec3(0.0, 2.0, 0.0), Vec3(1, 0, 0), 10, false));

		sys.m_springs.push_back(Roman::Spring(0, 1, 40, 1.0));

		Roman::Midpoint integrator;
		integrator.setSystem(&sys);
		for (size_t i = 0; i < 3; i++)
		{
			std::cout << "iteration: " << i << std::endl;
			sys.printOutParticles();
			integrator.doStep(0.1);
		}
	}
	system("pause");
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	cout << "Testcase changed to: ";
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case EULER:
		cout << "Euler !\n";
		m_particleIntegrator = std::make_unique<EulerParticleIntegrator>();
		break;
	case LEAPFROG:
		cout << "Leapfrog (not implemented. using euler instead)!\n";
		m_particleIntegrator = std::make_unique<EulerParticleIntegrator>();
		break;
	case MIDPOINT:
		cout << "Midpoint !\n";
		//m_particleIntegrator = std::make_unique<MidpointParticleIntegrator>();
		break;
	default:
		cout << "Undefined Testcase!\n";
		break;
	}

	setupScene(m_romanSys);
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		//m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	else {
		//m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{// handling different cases
	case EULER:
		break;
	case LEAPFROG:
		// not implemented
		break;
	case MIDPOINT:
		cout << "Midpoint !\n";
		break;
	default:
		cout << "Undefined Testcase!\n";
		break;
	}
	m_worldState = m_particleIntegrator->GetNextSimulationStep(m_worldState, timeStep);

	m_romanInt.doStep(timeStep);
	//std::cout << "Doing step !" << std::endl;
}

/*
void MassSpringSystemSimulator::drawSomeRandomObjects()
{
    std::mt19937 eng;
    std::uniform_real_distribution<float> randCol( 0.0f, 1.0f);
    std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
    for (int i=0; i<m_iNumSpheres; i++)
    {
		DUC->setUpLighting(Vec3(),0.4*Vec3(1,1,1),100,0.6*Vec3(randCol(eng),randCol(eng), randCol(eng)));
		DUC->drawSphere(Vec3(randPos(eng),randPos(eng),randPos(eng)),Vec3(m_fSphereSize, m_fSphereSize, m_fSphereSize));
    }
}

void MassSpringSystemSimulator::drawMovableTeapot()
{
	DUC->setUpLighting(Vec3(),0.4*Vec3(1,1,1),100,0.6*Vec3(0.97,0.86,1));
	DUC->drawTeapot(m_vfMovableObjectPos,m_vfRotate,Vec3(0.5,0.5,0.5));
}

void MassSpringSystemSimulator::drawTriangle()
{
	DUC->DrawTriangleUsingShaders();
}*/

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	this->renderWorldParticles(this->m_worldState);
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	this->m_selectedParticle = this->findClosesPoint(this->m_worldState, x, y);

	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

Vec3 MassSpringSystemSimulator::screenToRay(const float px, const float py)
{
	auto screenSize = this->getScreenSize();
	DirectX::XMMATRIX projectionMatrix, viewMatrix, inverseViewMatrix;
	projectionMatrix = this->DUC->g_camera.GetProjMatrix();
	viewMatrix = this->DUC->g_camera.GetViewMatrix();
	
	auto pixelVector = DirectX::XMVectorSet(px, py, 0.0, 1.0);
	auto resultVector = DirectX::XMVector3Unproject(pixelVector, 0.0, 0.0, screenSize.x, screenSize.y, 0.0, 1.0, projectionMatrix, viewMatrix, DirectX::XMMatrixIdentity());

	return Vec3(DirectX::XMVectorGetX(resultVector), DirectX::XMVectorGetY(resultVector), DirectX::XMVectorGetZ(resultVector));
}

Vec3 MassSpringSystemSimulator::getCameraPosition()
{
	DirectX::XMMATRIX projectionMatrix, viewMatrix, inverseViewMatrix;
	viewMatrix = this->DUC->g_camera.GetViewMatrix();
	inverseViewMatrix = DirectX::XMMatrixInverse(nullptr, viewMatrix);
	auto cameraOrigin = DirectX::XMVectorSet(0, 0, 0, 1.0);
	auto cameraPositionVector = DirectX::XMVector4Transform(cameraOrigin, inverseViewMatrix);
	auto& cpv = cameraPositionVector;
	return Vec3(DirectX::XMVectorGetX(cpv), ::XMVectorGetY(cpv), ::XMVectorGetZ(cpv));
}

Vec3 MassSpringSystemSimulator::getScreenSize()
{
	D3D11_VIEWPORT viewport;
	UINT viewportsCount = 1;
	this->DUC->g_pd3dImmediateContext->RSGetViewports(&viewportsCount, &viewport);
	return Vec3(viewport.Width,viewport.Height,1.0);
}

Vec3 MassSpringSystemSimulator::pointToScreen(const Vec3 point3D)
{
	auto pointVector = DirectX::XMVectorSet(point3D.x, point3D.y, point3D.z, 1.0);
	auto viewMat = this->DUC->g_camera.GetViewMatrix();
	auto projectionMat = this->DUC->g_camera.GetProjMatrix();
	auto screenSize = this->getScreenSize();
	auto screenVector = DirectX::XMVector3Project(pointVector, 0, 0, screenSize.x, screenSize.y, 0.0, 1.0, projectionMat, viewMat, DirectX::XMMatrixIdentity());

	return Vec3(DirectX::XMVectorGetX(screenVector), DirectX::XMVectorGetY(screenVector),0);
}

size_t MassSpringSystemSimulator::findClosesPoint(const WorldState & world, float px, float py)
{
	float distance = 1e10;
	float threshold = 40;
	// Always return at least the first point
	size_t pickedIndex = 0;
	for(size_t index = 0; index < world.particles.size(); index++)
	{
		auto& particle = world.particles[index];
		auto currentDistance = norm(this->pointToScreen(particle.position) - Vec3(px, py, 0.0));
		if (currentDistance < distance)
		{
			distance = currentDistance;
			pickedIndex = index;
		}
	}
	if (distance > threshold)
	{   // if the closest point further than threshold, then abort
		return -1;
	}
	// return the index of the closes point
	return pickedIndex;
}

void MassSpringSystemSimulator::renderWorldParticles(const WorldState & world)
{
	for (size_t index = 0; index < m_romanSys.m_particles.size(); index++)
	{
		auto& particle = m_romanSys.m_particles[index];

		auto position = DirectX::XMVectorSet(particle.position.x, particle.position.y, particle.position.z,1.0);
		auto scale = DirectX::XMVectorSet(particle.mass, particle.mass, particle.mass, 1.0);
		scale = DirectX::XMVectorScale(scale, 0.1);
		Vec3 color = Vec3(1.0);
		if (m_selectedParticle == index)
		{
			color = Vec3(1.0, 0.0, 0.0);
		}
		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, color);
		this->DUC->drawSphere(position, scale);
	}

	for (auto& spring: m_romanSys.m_springs)
	{
		auto& p1 = m_romanSys.m_particles[spring.start];
		auto& p2 = m_romanSys.m_particles[spring.end];

		// Detect line length 
		auto lineLength = norm(p1.position - p2.position);
		auto ratio = lineLength / spring.length;
		// Map line length to line color
		// - close to 0 => red
		// - close to 1 => white
		// - close to 2 => blue
		static Vec3 red = Vec3(1.0, 0.0, 0.0);
		static Vec3 white = Vec3(1.0, 1.0, 1.0);
		static Vec3 blue = Vec3(0.0, 0.0, 1.0);
		Vec3 color = Vec3();
		if (ratio > 2.0)
		{	// clamp at <0.0,2.0>
			ratio = 2.0;
		}
		// Hack: non-lineary ehance red/blue color compared to white
		float nonLinearRatio = 4.0;
		if (ratio <= 1.0)
		{
			ratio = pow(ratio, nonLinearRatio);
			color = red * (1.0 - ratio) + white * ratio;
		}
		else {
			ratio -= 1.0;
			ratio = pow(ratio, 1.0/nonLinearRatio);
			color = white * (1.0 - ratio) + blue * ratio;
		}

		this->DUC->beginLine();
		this->DUC->drawLine(p1.position,color,p2.position,color);
		this->DUC->endLine();
		
	}
	
}

void MassSpringSystemSimulator::setMass(float mass)
{
	for(auto& particle : m_worldState.particles)
	{
		particle.mass = mass;
	}
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
	for (auto& spring : m_worldState.springs)
	{
		spring.stiffness = stiffness;
	}
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_particleIntegrator->SetDampingFactor(damping);
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	m_worldState.particles.push_back(Particle(position, Velocity, m_fMass, isFixed));
	return m_worldState.particles.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	m_worldState.springs.push_back(Spring(ParticleHandle(masspoint1), ParticleHandle(masspoint2), initialLength, m_fStiffness));
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_worldState.particles.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_worldState.springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return m_worldState.particles[index].position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return m_worldState.particles[index].velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_particleIntegrator->AddGlobalForce(force);
}
