#pragma once
#include <glm/glm.hpp>

class Particle {
public:
	Particle(glm::vec3 _pos, glm::vec3 _vel);
	inline void SetPosition(glm::vec3 _pos) {
		position = _pos;
	}
	inline void SetVelocity(glm::vec3 _vel) {
		velocity = _vel;
	}

private:
	glm::vec3 position;
	glm::vec3 velocity;
};