#pragma once
#include "Particle.h"
#include <vector>

#define PARTICLE_COUNT 5000

class ParticleSystem {
public:
	ParticleSystem(){}

	void SetParticle(int index, glm::vec3 _pos, glm::vec3 _vel)
	{

		particlePositions[index].x = _pos.x;
		particlePositions[index].y = _pos.y;
		particlePositions[index].z = _pos.z;

		particleVelocities[index].x = _vel.x;
		particleVelocities[index].y = _vel.y;
		particleVelocities[index].z = _vel.z;

	}

	inline float* ParticlesPtr() {
		return &particlePositions[0].x;
	}

	
private:
	glm::vec3 particlePositions[PARTICLE_COUNT];
	glm::vec3 particleVelocities[PARTICLE_COUNT];
};