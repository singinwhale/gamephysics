#pragma once
#include <vector>
#include "../ParticleSimulationTypes.h"

namespace import
{
	class SpringsImporter
	{
	public:
		virtual void GetPrimitives(std::vector<Particle>& outParticles, std::vector<Spring>& outSprings) = 0;
	};


	struct Edge
	{
		union
		{
			std::size_t vertexIndices[2] = {SIZE_MAX, SIZE_MAX};
			struct
			{
				std::size_t vertexA;
				std::size_t vertexB;
			};
		};
	};

	struct Face
	{
		union
		{
			std::size_t vertexIndices[3] = { SIZE_MAX,SIZE_MAX, SIZE_MAX };
			struct {
				std::size_t vertexA;
				std::size_t vertexB;
				std::size_t vertexC;
			};
		};
	};
}