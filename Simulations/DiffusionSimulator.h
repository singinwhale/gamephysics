#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

//impement your own grid class for saving grid data
class Grid {
public:
	using cell_t = double;
	// Construtors
	Grid(const size_t x = 16, const size_t y = 16);

	cell_t& getCellAt(const size_t x, const size_t y) { return m_grid[m_width*y + x]; }
	size_t getWidth() const { return m_width; }
	size_t getHeight() const { return m_height; }
	void fill(float value) { std::fill(m_grid.begin(), m_grid.end(),value); }
private:
	// Attributes
	size_t m_width;
	size_t m_height;
	std::vector<cell_t> m_grid;
};

class SimulationStatus
{
public:
	SimulationStatus() { m_alpha = 1.0; }
	// Alpha coef
	float m_alpha;
	// Temperature at t
	std::unique_ptr<Grid> m_grid;
	// Temperature at t-1
	std::unique_ptr<Grid> m_previous;
};

class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();
	Grid* diffuseTemperatureExplicit(SimulationStatus& simulation, float deltaTime);
	void diffuseTemperatureImplicit();

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid *T; //save results of every time step

	SimulationStatus m_status;

};

#endif