#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

Grid::Grid(const size_t x, const size_t y) {
	// Allocated & fill grid with 0.0
	this->m_grid = std::vector<cell_t>(x*y, 0.0);
	this->m_width = x;
	this->m_height = y;
}


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// to be implemented
	this->m_status.m_grid = std::make_unique<Grid>(Grid(3, 3));
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// to be implemented
	TwAddVarCB(DUC->g_pTweakBar, "m", TW_TYPE_INT32, &m_iNumSpheres, "min=1");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	//to be implemented
	//

	switch (m_iTestCase)
	{
	case 0:
		{
			cout << "Reset Explicit solver!\n";
			auto& grid = this->m_status.m_grid;
			// Fill with zero
			grid->fill(0.0);
			grid->getCellAt(grid->getWidth() / 2, grid->getHeight() / 2) = 1000.0;
		}
	break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}

}

Grid* DiffusionSimulator::diffuseTemperatureExplicit(SimulationStatus& simulation, float deltaTime) {//add your own parameters
	const size_t size[2] = { simulation.m_grid->getWidth(), simulation.m_grid->getHeight() };
	auto& newT = std::make_unique<Grid>(Grid(size[0],size[1]));
	
	auto gridPtr = simulation.m_grid.get();
	//make sure that the temperature in boundary cells stays zero
	const auto& T = [gridPtr](int x, int y) { 
		if ((x < 0 || y < 0) || (x >= gridPtr->getWidth() || y >= gridPtr->getHeight()))
			return 0.0f; 
		return gridPtr->getCellAt(x, y); 
	};
	//const auto& Told = [&](int x, int y) { if ((x < 0 || y < 0) || (x >= simulation.m_grid->getWidth() || y >= simulation.m_grid->getHeight())) return 0.0; else simulation.m_previous->getCellAt(x, y); };
	const auto alpha = simulation.m_alpha;
	for (size_t y = 0; y < size[1]; y++)
	{
		for (size_t x = 0; x < size[0]; x++)
		{
			// compute dTdt: see slides to use forward method for time derivative and central method for dT2dx2 and dT2dy2
			auto dTdt = alpha * (T(x + 1, y) - 2 * T(x, y) + T(x - 1, y) + T(x, y + 1) - 2 * T(x, y) + T(x, y - 1)) * deltaTime + T(x, y);;
			// Integrate using Euler
			//auto newTcell = dTdt*deltaTime+T(x,y);
			// Store result
			newT->getCellAt(x, y) = dTdt;

			std::cout << "Cell diff [" << x << ", " << y << "]: " << (newT->getCellAt(x, y) - T(x, y)) << std::endl;
		}
	}
	simulation.m_previous = std::move(simulation.m_grid);
	simulation.m_grid = std::move(newT);
	return simulation.m_grid.get();
}

void setupB(std::vector<Real>& b) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < 25; i++) {
		b.at(i) = 0;
	}
}

void fillT() {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
}

void setupA(SparseMatrix<Real>& A, double factor) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < 25; i++) {
			A.set_element(i, i, 1); // set diagonal
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit() {//add your own parameters
	// solve A T = b
	// to be implemented
	const int N = 25;//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N);
	std::vector<Real> *b = new std::vector<Real>(N);

	setupA(*A, 0.1);
	setupB(*b);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT();//copy x to T
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit(this->m_status,timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit();
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization

	switch (m_iTestCase)
	{
	case 0:
		for (size_t x = 0; x < T->getWidth(); x++)
		{
			for (size_t y = 0; y < T->getHeight(); y++)
			{
				const float scX = 1.0 / T->getWidth();
				const float scY = 1.0 / T->getHeight();
				auto temp = T->getCellAt(x, y);
				auto pos = Vec3(x*scX, y*scY, 0.0);
				auto size = Vec3(scX, scY, 0.1);

				auto const maxTemp = 100.0;
				// Interpolation parameter t
				auto t = temp / maxTemp;
				// Interpolate between white and red color
				auto color = Vec3(1.0, 0.0, 0.0)*t + (1.0 - t)*Vec3(1.0);
				// Draw
				this->DUC->setUpLighting(Vec3(0.0), Vec3(1.0), 0.1, color);
				this->DUC->drawSphere(pos, size);
			}
		}
	}
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
