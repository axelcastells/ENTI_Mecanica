#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\gtc\matrix_transform.hpp>
#include <vector>
#include <thread>
#include <iostream>
#include <string>

#include "Tools.h"

#define DEG2RAD 0.0174532924
#define RAD2DEG 57.29578

#define SPHERE_IBODY_MATRIX(m, r) glm::mat3((m * (r*r)) * (2/5))

#define SPHERES_COUNT 3
#define CAGE_SIZE 10
#define CAGE_PLANES_COUNT 6
#define SPHERE_MASS 1
#define SPHERE_RADIUS 1

#define COLLISION_MAX_ITERATIONS 20
#define COLLISION_TOLERANCE 0.1f;

//GUI
#define GRAVITY_MIN 0
#define GRAVITY_MAX 10
//Sphere 1
#define SPHERE_MASS_MIN 1
#define SPHERE_MASS_MAX 100
#define SPHERE_RADIUS_MIN 0.1f
#define SPHERE_RADIUS_MAX 10

#define COLLISION_TOLERANCE 10
#define COLLISION_ACCURACY 10

#pragma region Parameters
static float GRAVITY_FORCE = 9.81f;					// Gravity Force
static glm::vec3 GRAVITY_VECTOR = { 0, -1, 0 };		// Gravity Vector
static float RESTITUTION_FACTOR = 1.f;				// Elasticity
static bool SIMULATE = false;						// Simulation Boolean
static float TIME_SCALE = 1.f;						// Time Scale

#pragma endregion


static float counter;
void PhysicsCleanup();
void ResetSimulation();

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


struct Collider {
	glm::vec3 contactPoint;
	virtual bool checkCollision(const glm::vec3& next_pos, float radius) = 0;
	//virtual bool findContactPoint(); // TO DO
};

struct PlaneCol : Collider {
	glm::vec3 position, planeNormal;

	PlaneCol(){}
	PlaneCol(glm::vec3 _pos, glm::vec3 _norm) : position(_pos), planeNormal(_norm){}

	void getPlane(glm::vec3& normal, float& d) {
		normal = planeNormal;
		d = glm::dot(-normal, position);
	}

	bool checkCollision(const glm::vec3& next_pos, float radius) override {
		//return false;
		glm::vec3 norm;
		float d;
		getPlane(norm, d);


		float distance = ((next_pos.x * norm.x) + (next_pos.y * norm.y) + (next_pos.z * norm.z) + d);
		contactPoint = next_pos * norm;

		// TODO: FIND REAL COLLISION POINT

		if (distance <= radius) return true;
		else return false;

	}
};

struct RigidSphere : Collider {
	//...
	float mass, rad, j;
	glm::mat3 rotation;
	glm::vec3 contactPoint;
	glm::vec3 prevPosition, position, linearMomentum, angularMomentum, velocity, angularVelocity;
	

	glm::mat3 IBody() { return SPHERE_IBODY_MATRIX(mass, rad); }

	RigidSphere() = delete;
	RigidSphere(glm::vec3 _pos, float _mass, float _rad, glm::vec3 _linearVel, glm::vec3 _angularVel) :
		mass(_mass), rad(_rad), position(_pos), linearMomentum(_linearVel), angularMomentum(_angularVel)
	{
		rotation = glm::mat3(1);
		prevPosition = position;
	}
	bool checkCollision(const glm::vec3& next_pos, float radius) override {
		//...
		if (glm::distance(position, next_pos) <= rad + radius) {
			
			contactPoint = position + (glm::normalize(position - next_pos) * rad);


			return true;
		}
		return false;
	}
};

void FindContactPoint(RigidSphere& sph, float dt, Collider* col, int accuracy, float tolerance) {
	// FIND TRUE CONTACT
	for (int i = 0; i < accuracy; i++) {
		if (col->checkCollision(sph.position, sph.rad)) {
			dt -= dt / 2;
		}
		else {
			dt += dt / 2;
		}

		sph.position = sph.prevPosition + dt * sph.velocity;
		if (glm::abs(glm::distance(sph.position, col->contactPoint) < tolerance)) break;
	}
}

// BOX
// X -5 / 5
// Y 0 / 10
// Z -5 / 5
std::vector<Collider*> colliders;

float computeImpulseCorrection(float massA, glm::vec3 ra, glm::mat3 invIa, float massB, glm::vec3 rb, glm::mat3 invIb, float vrel, float epsilon, glm::vec3 normal) {
	// "-vrel" representa la "inversa" de la velocitat real.
	if (massB < 0) {
		return-(1 + epsilon)* (vrel) / ((1 / massA) + glm::dot(normal, glm::cross((invIa * glm::cross(ra, normal)), ra)));
	}
	else {
		return -(1 + epsilon)* (vrel) / ((1 / massA) + (1 / massB) + glm::dot(normal, glm::cross((invIa * glm::cross(ra, normal)), ra)) +
			glm::dot(normal, glm::cross((invIb * glm::cross(rb, normal)), rb)));
	}
	
}

void updateColliders(Collider* A, Collider* B) {
	RigidSphere* sphA;
	RigidSphere* sphB;
	sphA = dynamic_cast<RigidSphere*>(A);
	sphB = dynamic_cast<RigidSphere*>(B);

	if (sphA != NULL && sphB != NULL) {
		sphA->linearMomentum = sphA->linearMomentum + sphA->j * glm::normalize(sphA->position - sphB->position);// glm::cross(sphA->position, (sphA->j*glm::normalize(sphA->position - sphB->position)));
		sphA->angularMomentum = sphA->angularMomentum + glm::cross(sphA->contactPoint - sphA->position, sphA->j * glm::normalize(sphA->position - sphB->position));

		sphB->linearMomentum = sphB->linearMomentum + sphB->j * glm::normalize(sphB->position - sphA->position);
		sphB->angularMomentum = sphB->angularMomentum + glm::cross(sphB->contactPoint - sphB->position, sphB->j * glm::normalize(sphA->position - sphA->position));
	}
	else if (sphA != NULL) {
		PlaneCol* plane = dynamic_cast<PlaneCol*>(B);
		if (plane != NULL) {
			sphA->linearMomentum = sphA->linearMomentum + sphA->j * glm::normalize(sphA->position - plane->contactPoint);// glm::cross(sphA->position, (sphA->j*glm::normalize(sphA->position - sphB->position)));
			sphA->angularMomentum = sphA->angularMomentum + glm::cross(sphA->contactPoint - sphA->position, sphA->j * glm::normalize(sphA->position - plane->contactPoint));
		}
		plane = 0;
		delete(plane);
	}

	sphA = 0;
	sphB = 0;
	delete(sphA);
	delete(sphB);
	
}

void euler(float dt, RigidSphere& sph) {
	glm::vec3 force = (GRAVITY_FORCE * sph.mass * GRAVITY_VECTOR);
	glm::vec3 torque = glm::cross(sph.contactPoint - sph.position, force);
	sph.linearMomentum = sph.linearMomentum + dt*force;
	sph.angularMomentum = sph.angularMomentum + dt*torque;

	sph.velocity = sph.linearMomentum / sph.mass;
	sph.position = sph.position + dt* sph.velocity;

	glm::mat3 inertiaTensorInverse = sph.rotation * glm::inverse(sph.IBody()) * glm::transpose(sph.rotation);


	// COLLISION CHECKS
	for (int i = 0; i < SPHERES_COUNT + CAGE_PLANES_COUNT; i++) {
		if (colliders[i] == &sph);
		else 
		{
			if (colliders[i]->checkCollision(sph.position, sph.rad)) 
			{
				RigidSphere* sphere;
				sphere = dynamic_cast<RigidSphere*>(colliders[i]);
				if (sphere != NULL)
				{
					FindContactPoint(sph, dt, colliders[i], COLLISION_ACCURACY, COLLISION_TOLERANCE);
					
					std::cout << "Sphere-Sphere Collision!" << std::endl;
					// SPHERES COLLISION
					glm::mat3 newIntertiaTensor = sph.rotation * glm::inverse(sph.IBody()) * glm::transpose(sph.rotation);
					
					// TODO: Calculate vrel
					glm::vec3 pa = sph.velocity + glm::cross(sph.angularVelocity, sph.contactPoint - sph.position);
					glm::vec3 pb = sphere->velocity + glm::cross(sphere->angularVelocity, sphere->contactPoint - sphere->position);
					float vrel = glm::dot(glm::normalize(sph.position - sph.contactPoint), pa - pb);
					float j = computeImpulseCorrection(sph.mass, sph.position, inertiaTensorInverse, sphere->mass, sphere->position, newIntertiaTensor, 
						vrel, RESTITUTION_FACTOR, glm::normalize(sph.position - sph.contactPoint));
					
					sph.j = j;
					sphere->j = -j;
					
					//updateColliders(&sph, sphere);

					
				}
				else
				{
					sphere = 0;
					delete(sphere);
					PlaneCol* plane;
					plane = dynamic_cast<PlaneCol*>(colliders[i]);
					if (plane != NULL) 
					{
						FindContactPoint(sph, dt, colliders[i], COLLISION_ACCURACY, COLLISION_TOLERANCE);

						// PLANES COLLISION
						glm::vec3 pNorm;
						float d;
						plane->getPlane(pNorm, d);

						std::cout << "Sphere-Plane Collision!" << std::endl;
						glm::mat3 newIntertiaTensor = sph.rotation * glm::inverse(sph.IBody()) * glm::transpose(sph.rotation);

						//// TODO: Calculate vrel
						glm::vec3 pa = sph.velocity + glm::cross(sph.angularVelocity, sph.contactPoint - sph.position);

						float vrel = glm::dot(glm::normalize(sph.position - sph.contactPoint), pa);
						float j = computeImpulseCorrection(sph.mass, sph.position, inertiaTensorInverse, -1, plane->contactPoint, newIntertiaTensor,
							vrel, RESTITUTION_FACTOR, glm::normalize(sph.position - sph.contactPoint));

						//updateColliders(&sph, plane);

					}
					plane = 0;
					delete(plane);
				}
			}
		}

	}

	sph.prevPosition = sph.position;

	glm::vec3 angularVel = inertiaTensorInverse * sph.angularMomentum;
	sph.rotation += dt * (angularVel * sph.rotation);
}


// Boolean variables allow to show/hide the primitives
bool renderSphere = true;
bool renderCapsule = false;
bool renderParticles = false;
bool renderMesh = false;
bool renderFiber = false;
bool renderCube = false;

//You may have to change this code
void renderPrims() {
	Box::drawCube();
	Axis::drawAxis();


	if (renderSphere)
	{
		for (int i = 0; i < SPHERES_COUNT; i++)
		{
			Sphere::updateSphere(dynamic_cast<RigidSphere*>(colliders[i])->position, dynamic_cast<RigidSphere*>(colliders[i])->rad);
			Sphere::drawSphere();
		}
	}
	if (renderCapsule)
		Capsule::drawCapsule();

	if (renderParticles) {
		int startDrawingFromParticle = 0;
		int numParticlesToDraw = Particles::maxParticles;
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
		ImGui::Checkbox("Play/Pause Simulation", &SIMULATE);
		if(ImGui::Button("Reset Simulation", ImVec2(130, 20))) 
		{
			ResetSimulation();
		}

		ImGui::DragFloat("Gravity Value", &GRAVITY_FORCE, 0.1f, GRAVITY_MIN, GRAVITY_MAX, "%.3f");
		ImGui::DragFloat3("Gravity Vector", &GRAVITY_VECTOR.x, 0.1f, 0, 1, "%.3f");
		ImGui::DragFloat("Elasticity", &RESTITUTION_FACTOR, 0.05f, 0.0f, 1.0f, "%.3f");


		for (int i = 0; i < SPHERES_COUNT; i++) {
			std::string name("Sphere " + std::to_string(i));
			RigidSphere* sphere = dynamic_cast<RigidSphere*>(colliders[i]);
			ImGui::Text(name.c_str());
			ImGui::DragFloat(std::string("Mass " + std::to_string(i)).c_str(), &sphere->mass, 0.1f, SPHERE_MASS_MIN, SPHERE_MASS_MAX, "%.3f");
			ImGui::DragFloat(std::string("Radius " + std::to_string(i)).c_str(), &sphere->rad, 0.1f, SPHERE_RADIUS_MIN, SPHERE_RADIUS_MAX, "%.3f");

		}

	}
	// .........................
	
	ImGui::End();

	// Example code -- ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	bool show_test_window = false;
	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

void PhysicsReset() {
	for (int i = 0; i < SPHERES_COUNT; i++) {
		colliders.push_back(new RigidSphere(glm::vec3((CAGE_SIZE / 2) - (Tools::Random() * (CAGE_SIZE / 2)),
			Tools::Random() * CAGE_SIZE,
			(CAGE_SIZE / 2) - (Tools::Random() * (CAGE_SIZE / 2))),
			SPHERE_MASS, SPHERE_RADIUS, glm::vec3(Tools::Map(Tools::Random(), 0, 1, -3, 3), Tools::Map(Tools::Random(), 0, 1, -3, 3), Tools::Map(Tools::Random(), 0, 1, -3, 3)),
			glm::vec3(Tools::Map(Tools::Random(), 0, 1, -3, 3), Tools::Map(Tools::Random(), 0, 1, -3, 3), Tools::Map(Tools::Random(), 0, 1, -3, 3))));

	}
	colliders.push_back(new PlaneCol(glm::vec3(0, 0, 0), glm::vec3(0, 1, 0)));
	colliders.push_back(new PlaneCol(glm::vec3(0, CAGE_SIZE, 0), glm::vec3(0, -1, 0)));
	colliders.push_back(new PlaneCol(glm::vec3(-CAGE_SIZE / 2, 0, 0), glm::vec3(1, 0, 0)));
	colliders.push_back(new PlaneCol(glm::vec3(CAGE_SIZE / 2, 0, 0), glm::vec3(-1, 0, 0)));
	colliders.push_back(new PlaneCol(glm::vec3(0, 0, -CAGE_SIZE / 2), glm::vec3(0, 0, 1)));
	colliders.push_back(new PlaneCol(glm::vec3(0, 0, CAGE_SIZE / 2), glm::vec3(0, 0, -1)));
}

void ResetSimulation() {
	counter = 0;
	PhysicsCleanup();
	PhysicsReset();
}
void PhysicsInit() {
	// Do your initialization code here...
	srand((unsigned)time(NULL));
	PhysicsReset();
	// ...................................
}

void PhysicsUpdate(float dt) {
	if (SIMULATE) {

		dt *= TIME_SCALE;
		// Do your update code here...

		counter += dt;
		if (counter >= 15) {
			counter = 0;
			ResetSimulation();
		}

		for (int i = 0; i < SPHERES_COUNT; i++) {
			euler(dt, dynamic_cast<RigidSphere&>(*colliders[i]));
		}
		// ...........................
	}
}

void PhysicsCleanup() {
	// Do your cleanup code here...
	colliders.clear();
	// ............................
}