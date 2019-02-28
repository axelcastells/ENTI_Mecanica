#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\gtc\matrix_transform.hpp>
#include "ParticleSystem.h"
#include "Tools.h"

#pragma region Program

namespace Box {
	void drawCube();
}
namespace Axis {
	void drawAxis();
}

namespace Sphere {
	extern void updateSphere(glm::vec3 pos, float radius = 1.f);
	extern void drawSphere();
}
namespace Capsule {
	extern void updateCapsule(glm::vec3 posA, glm::vec3 posB, float radius = 1.f);
	extern void drawCapsule();
}
namespace Particles {
	extern const int maxParticles;
	extern void updateParticles(int startIdx, int count, float* array_data);
	extern void drawParticles(int startIdx, int count);
}
namespace Mesh {
	extern const int numCols;
	extern const int numRows;
	extern void updateMesh(float* array_data);
	extern void drawMesh();
}
namespace Fiber {
extern const int numVerts;
	extern void updateFiber(float* array_data);
	extern void drawFiber();
}
namespace Cube {
	extern void updateCube(const glm::mat4& transform);
	extern void drawCube();
}



// Boolean variables allow to show/hide the primitives
bool renderSphere = false;
bool renderCapsule = false;
bool renderParticles = true;
bool renderMesh = false;
bool renderFiber = false;
bool renderCube = false;

//You may have to change this code
void renderPrims() {
	Box::drawCube();
	Axis::drawAxis();


	if (renderSphere)
		Sphere::drawSphere();
	if (renderCapsule)
		Capsule::drawCapsule();

	if (renderParticles) {
		int startDrawingFromParticle = 0;
		int numParticlesToDraw = PARTICLE_COUNT;// Particles::maxParticles;
		Particles::drawParticles(startDrawingFromParticle, numParticlesToDraw);
	}

	if (renderMesh)
		Mesh::drawMesh();
	if (renderFiber)
		Fiber::drawFiber();

	if (renderCube)
		Cube::drawCube();
}


void GUI() {
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);

	// Do your GUI code here....
	{	
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);//FrameRate
		
	}
	// .........................
	
	ImGui::End();

	// Example code -- ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	bool show_test_window = 1;
	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}
#pragma endregion
// ------------------------------------------------------------------------------------------
#define GRAVITY_FORCE 9.81f
#define BOUNCE_ELASTICITY 0.8f

// Force Actuators
struct ForceActuator { 
	virtual glm::vec3 computeForce(float mass, const glm::vec3& position) = 0; 
};

struct GravityForce : ForceActuator {
	glm::vec3 computeForce(float mass, const glm::vec3& position) override {
		return glm::vec3(0, mass * -GRAVITY_FORCE, 0);
	}
};

struct PositionalGravityForce : ForceActuator {
	glm::vec3 computeForce(float mass, const glm::vec3& position) override {

	}
};
// ------------------------------------------------------------------------------------------

// Colliders
struct Collider {
	virtual bool checkCollision(const glm::vec3& prev_pos, const glm::vec3& next_pos) = 0; 
	virtual void getPlane(glm::vec3& normal, float& d) = 0; 
	void computeCollision(const glm::vec3& old_pos, const glm::vec3& old_vel, glm::vec3& new_pos, glm::vec3& new_vel) 
	{
		if (checkCollision(old_pos, new_pos)) {
			glm::vec3 normal;
			float d;
			getPlane(normal, d);

			new_pos = new_pos - ((1 + BOUNCE_ELASTICITY) * glm::dot(normal, new_pos) + d) * normal;
			new_vel = new_vel - ((1 + BOUNCE_ELASTICITY) * glm::dot(normal, new_vel)) * normal;
		}
	}
};
struct PlaneCol : Collider {
	//...
	glm::vec3 planePosition, planeNormal;
	PlaneCol(glm::vec3 pos, glm::vec3 norm) {
		planePosition = pos;
		planeNormal = norm;
	}
	bool checkCollision(const glm::vec3& prev_pos, const glm::vec3& next_pos) override {
		glm::vec3 direction = (next_pos - prev_pos);
		float magnitude = glm::sqrt(glm::pow(direction.x, 2) + glm::pow(direction.x, 2) + glm::pow(direction.x, 2));
		glm::vec3 normalizedDirection = direction / magnitude;

		glm::vec3 norm;
		float d;
		getPlane(norm, d);

		float distance = glm::dot(norm, (next_pos - planePosition));
		distance *= -1;

		if (distance <= 0) return true;
		else return false;

	}
	void getPlane(glm::vec3& normal, float& d) override {
		normal = planeNormal;
		d = glm::dot(-planeNormal, planePosition);
	}
};
struct SphereCol : Collider {
	//...
	bool checkCollision(const glm::vec3& prev_pos, const glm::vec3& next_pos) override {
		return false;
	}
	void getPlane(glm::vec3& normal, float& d) override {

	}
};
struct CapsuleCol : Collider {
	//...
	bool checkCollision(const glm::vec3& prev_pos, const glm::vec3& next_pos) override {

	}
	void getPlane(glm::vec3& normal, float& d) override {

	}
};

void euler(float dt, ParticleSystem& particles, const std::vector<Collider*>& colliders, const std::vector<ForceActuator*>& force_acts) {
	for (int i = 0; i < PARTICLE_COUNT; i++) {
		glm::vec3 forces;
		for (int j = 0; j < force_acts.size(); j++) {
			forces += force_acts[j]->computeForce(GLOBAL_PARTICLE_MASS, particles.particlePositions[i]);
		}
		glm::vec3 accel = glm::vec3(forces.x / GLOBAL_PARTICLE_MASS, forces.y / GLOBAL_PARTICLE_MASS, forces.z / GLOBAL_PARTICLE_MASS);
		particles.particleVelocities[i] += (dt * accel);
		particles.particlePositions[i] += (dt * particles.particleVelocities[i]);

		for (int j = 0; j < colliders.size(); j++) {
			colliders[j]->computeCollision(particles.particlePositions[i], particles.particleVelocities[i], 
											particles.particlePositions[i], particles.particleVelocities[i]);
		}

	}
}

ParticleSystem ps = ParticleSystem();
std::vector<ForceActuator*> forces;
std::vector<Collider*> colliders;

void PhysicsInit() {
	// Do your initialization code here...
	for (int i = 0; i < PARTICLE_COUNT; i++) {
		float f = Tools::Random();
		glm::vec3 newPos(
			Tools::Map(Tools::Random(), 0, 1, -5, 5), 
			Tools::Map(Tools::Random(), 0, 1, 5, 10), 
			Tools::Map(Tools::Random(), 0, 1, -5, 5));

		//glm::vec3 newVel(
		//	Tools::Random() * 5,
		//	Tools::Random() * 5,
		//	Tools::Random() * 5);

		ps.SetParticle(i, newPos, glm::vec3(0));
	}

	forces.push_back(new GravityForce());
	colliders.push_back(new PlaneCol(glm::vec3(0,0,0), glm::vec3(0,-1,0)));
	// ...................................
}

void PhysicsUpdate(float dt) {
	// Do your update code here...

	euler(dt, ps, colliders, forces);

	Particles::updateParticles(0, PARTICLE_COUNT, ps.ParticlesPtr());
	// ...........................
}

void PhysicsCleanup() {
	// Do your cleanup code here...
	// ............................
}