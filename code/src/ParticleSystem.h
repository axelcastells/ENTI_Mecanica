#pragma once
#include <vector>

#define MAX_PARTICLES 5000

class ParticleSystem {
public:
	ParticleSystem(){}
	~ParticleSystem() {

	}
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

	glm::vec3 particlePositions[MAX_PARTICLES];
	glm::vec3 particleVelocities[MAX_PARTICLES];
private:

};