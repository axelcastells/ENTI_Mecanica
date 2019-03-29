#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\gtc\matrix_transform.hpp>
#include "ParticleSystem.h"
#include "FiberStraw.h"
#include "Tools.h"
#include "glm/gtx/intersect.hpp"
#include <iostream>

#pragma region GlobalData
//Colisions
static bool SPHERE_COLLISION = true;
static bool CAPSULE_COLLISION = true;
//Velocitat i Activacio del funcionament
static bool SIMULATE_FIBERS = false;
static bool SIMULATE_PARTICLES = false;
static bool GRAVITY_ACTIVE = true;
static bool POSITIONAL_GRAVITY_ACTIVE = true;
static float TIME_FACTOR = 1.0f;
//Masa de les particules
static float FIBER_PARTICLE_MASS = 1.f;
static float PARTICLE_MASS = 1.f;
// Factors de gravetat
static float GRAVITY_FORCE = 9.81f;
static glm::vec3 GRAVITY_VECTOR = { 0,-1,0 };
//Friccio i Elasticitat
static float BOUNCE_ELASTICITY = 0.8f;
static float FRICTION_FACTOR = .2f;

static float FIBER_ELASTICITY = 0.0f;
static float FIBER_DAMPLING = 0.f;
//Factors Esfera
static glm::vec3 SPHERE_POS = { 0,5,0 };
static float SPHERE_RAD = 1.5f;
static glm::vec3 SPHERE_ORBITAL_POINT = { 0,0,0 };
static float SPHERE_ORBITAL_MAGNITUDE = 2.f;
static float SPHERE_ORBITAL_SPEED = 4.f;
static float SPHERE_MASS = 1.f;
//Factors Capsula
static glm::vec3 CAPSULE_POS_A = { -2,1,1 };
static glm::vec3 CAPSULE_POS_B = { 2,1,1 };
static float CAPSULE_RAD = 1.f;

//Stats 
static float STRETCH = 0.0;
static float BEND = 0.0;
static float PLD = 0.0;

//Moviment esfera
static float SPHERE_TURN = 0.0;
static float SPHERE_SPEED = 0.0;

//Vent
static bool WIND_ACTIVE = false;
static float WIND_FORCE = 0.0;

//defines pel GUI
#define minPmass 1.0
#define maxPmass 100.0

#define minStretch 5
#define maxStretch 1000

#define minSphmass 1.0
#define maxSphmass 100.0

#define minBend 5
#define maxBend 1000

#define minPLD 0.1
#define maxPLD 1.0

#define Sphturnmin 0.1
#define Sphturnmax 5.0

#define Sphspeedmin 0.1
#define Sphspeedmax 2.0

#define minSphposX -5.0
#define maxSphposX 5.0
#define minSphposY 0.0
#define maxSphposY 10.0
#define minSphposZ -5.0
#define maxSphposZ 5.0

#define minSphrad 0.1
#define maxSphrad 5.0

#define minCapposA_X 0.0
#define maxCapposA_X 10.0
#define minCapposA_Y 0.0
#define maxCapposA_Y 10.0
#define minCapposA_Z 0.0
#define maxCapposA_Z 10.0

#define minCapposB_X -5.0
#define maxCapposB_X 5.0
#define minCapposB_Y 0.0
#define maxCapposB_Y 10.0
#define minCapposB_Z -5.0
#define maxCapposB_Z 5.0

#define minCaprad 0.1
#define maxCaprad 5.0

#define minGrabAccel 0.0
#define maxGrabAccel 9.81f

#define minWindAccel 0.0
#define maxWindAccel 10.0
#pragma endregion

#pragma region Program

void PhysicsRestart();
FiberSystem fs = FiberSystem();


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
bool renderSphere = true;
bool renderCapsule = false;
bool renderParticles = true;
bool renderMesh = false;
bool renderFiber = true;
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
		int numParticlesToDraw = MAX_PARTICLES;// Particles::maxParticles;
		Particles::drawParticles(startDrawingFromParticle, numParticlesToDraw);
	}

	if (renderMesh)
		Mesh::drawMesh();
	if (renderFiber)
	{
		for (int i = 0; i < fs.FibersCount(); i++) {
			Fiber::updateFiber(fs[i].DataPtr());
			Fiber::drawFiber();
		}
		
	}
		

	if (renderCube)
		Cube::drawCube();
}


void GUI() {
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);

	// Do your GUI code here....
	{
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);//FrameRate

		//PLAYING simulation
		//ImGui::Checkbox("Play/Pause Particles", &SIMULATE_PARTICLES);
		ImGui::Checkbox("Play/Pause Simulation", &SIMULATE_FIBERS);
		ImGui::DragFloat("Time Scale", &TIME_FACTOR, 0.01f, 0.1f, 1.0f, "%.3f");
		if (ImGui::Button("Reset", ImVec2(50, 20))) { //Trobar i reactivar la funcio que inicia la simulacio
			PhysicsRestart();
		}
		//ImGui::DragFloat("Particles Mass", &PARTICLE_MASS, 0.05f, minPmass, maxPmass, "%.3f");

		//Spring Parameters
		ImGui::Text("Spring Parameters");
		ImGui::DragFloat("Stretch", &STRETCH, 0.05f, minStretch, maxStretch, "%.3f");
		ImGui::DragFloat("Bend", &BEND, 0.05f, minBend, maxBend, "%.3f");
		ImGui::DragFloat("Particle Link Distance", &PLD, 0.05f, minPLD, maxPLD, "%.3f");

		//Elasticitat i Friccio Fibres
		ImGui::Text("Elasticity & Friction");
		ImGui::DragFloat("Elasticity", &BOUNCE_ELASTICITY, 0.05f, 0.0f, 1.0f, "%.3f");
		ImGui::DragFloat("Friction", &FRICTION_FACTOR, 0.05f, 0.0f, 1.0f, "%.3f");

		ImGui::Text("Colliders");
		//Sphere
		//Use Sphere Collider
		ImGui::Text("Sphere");
		ImGui::Checkbox("Sphere Collision", &SPHERE_COLLISION);
		ImGui::DragFloat("Sphere Y", &SPHERE_POS.y, 0.1, minSphposY, maxSphposY, "%.3f");
		ImGui::DragFloat("Sphere Turm Radius", &SPHERE_TURN, 0.1, Sphturnmin, Sphturnmax, "%.3f");
		ImGui::DragFloat("Sphere Turn Speed", &SPHERE_SPEED, 0.1, Sphspeedmin, Sphspeedmax, "%.3f");
		ImGui::DragFloat("Sphere Radius", &SPHERE_RAD, 0.05f, minSphrad, maxSphrad, "%.3f");

		ImGui::Text("Forces");
		//Use Gravity
		ImGui::Checkbox("Use Gravity", &GRAVITY_ACTIVE);
		ImGui::DragFloat("Gravity Accel", &GRAVITY_FORCE, 0.1f, minGrabAccel, maxGrabAccel, "%.3f");
		ImGui::Checkbox("Use Wind", &WIND_ACTIVE);
		ImGui::DragFloat("Wind Accel", &WIND_FORCE, 0.1f, minWindAccel, maxWindAccel, "%.3f");
	}
	// .........................

	ImGui::End();

	// Example code -- ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	bool show_test_window = 1;
	if (show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}
#pragma endregion

float Magnitude(glm::vec3 a, glm::vec3 b);

// ------------------------------------------------------------------------------------------


// Force Actuators
struct ForceActuator { 
	virtual glm::vec3 computeForce(float mass, const glm::vec3& position) = 0; 
};

struct GravityForce : ForceActuator {
	glm::vec3 computeForce(float mass, const glm::vec3& position) override {
		if(GRAVITY_ACTIVE) return GRAVITY_VECTOR * (mass * GRAVITY_FORCE);
		else return glm::vec3(0);
	}
};

struct PositionalGravityForce : ForceActuator {
	float *mass;
	glm::vec3 *position;
	PositionalGravityForce(float *_mass, glm::vec3 *_pos) {
		position = _pos;
		mass = _mass;
	}

	glm::vec3 computeForce(float _mass, const glm::vec3& _position) override {
		if(POSITIONAL_GRAVITY_ACTIVE) return ((GRAVITY_FORCE * _mass * *mass) / glm::pow(Magnitude(_position, *position), 2)) * (*position - _position) / Magnitude(_position, *position);
		else return glm::vec3(0);
	}
};
// ------------------------------------------------------------------------------------------

// Colliders
struct Collider {
	virtual bool checkCollision(const glm::vec3& prev_pos, const glm::vec3& next_pos) = 0; 
	virtual void getPlane(glm::vec3& normal, float& d) = 0; 
	void computeCollision(glm::vec3& old_pos, glm::vec3& new_pos) 
	{

	}
	void computeCollision(const glm::vec3& old_pos, const glm::vec3& old_vel, glm::vec3& new_pos, glm::vec3& new_vel) 
	{
		if (checkCollision(old_pos, new_pos)) {


			glm::vec3 normal;
			float d;
			getPlane(normal, d);



			//XOC PLA - PARTÍCULA | ELASTICITAT
			glm::vec3 pos_rebot = new_pos - (1 + BOUNCE_ELASTICITY) * (glm::dot(normal, new_pos) + d) * normal;
			glm::vec3 vel_rebot = new_vel - (1 + BOUNCE_ELASTICITY) * glm::dot(normal, new_vel) * normal;


			//XOC PLA - PARTÍCULA | FRICCIÓ
			glm::vec3 nVel, tVel;
			nVel = glm::dot(normal, old_vel) * normal;
			tVel = old_vel - nVel;

			vel_rebot = vel_rebot - (FRICTION_FACTOR * tVel);

			new_pos = pos_rebot;
			new_vel = vel_rebot;
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
		glm::vec3 norm;
		float d;
		getPlane(norm, d);

		float distance = -((next_pos.x * norm.x) + (next_pos.y * norm.y) + (next_pos.z * norm.z) + d);//glm::dot(norm, (next_pos - planePosition));

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
	glm::vec3 *position, collisionPoint;
	float *radius, *mass;

	SphereCol(glm::vec3 *_pos, float *_rad, float *_mass) {
		radius = _rad;
		position = _pos;
		mass = _mass;
	}
	
	bool checkCollision(const glm::vec3& prev_pos, const glm::vec3& next_pos) override {

		float distance_to_center = distance(next_pos,*position);
		
		if (distance_to_center <= *radius && SPHERE_COLLISION) {
			// equacio de 2n grau | Q = next_pos | P = prev_pos | C = centre de la esfera
			// a = Q*Q + P*P - 2*Q*P
			float a = glm::dot(next_pos, next_pos) + glm::dot(prev_pos, prev_pos) - 2 * glm::dot(next_pos, prev_pos);
			// b = 2*(P*Q + P*P - C*Q + C*P)
			float b = 2 * (glm::dot(prev_pos, next_pos) + glm::dot(prev_pos, prev_pos) - glm::dot(*position, next_pos) + glm::dot(*position,prev_pos));
			// c = C*C + P*P - r^2 - 2*C*P
			float c = glm::dot(*position, *position) + glm::dot(prev_pos, prev_pos) - glm::pow(*radius,2) - 2 * glm::dot(*position,prev_pos);
						
			//calculem alfa1 i alfa2 de la equació de 2n grau
			float alfa1 = (- b + sqrt(b*b - 4*a*c)) / 2*a;
			float alfa2 = (- b - sqrt(b*b - 4*a*c)) / 2*a;

			//calculem el vector entre la prev_pos i next_pos
			glm::vec3 vector_prev_next = next_pos - prev_pos;
			glm::vec3 S1 = prev_pos + vector_prev_next * alfa1;		//S1 = Point of surface 1
			glm::vec3 S2 = prev_pos + vector_prev_next * alfa2;		//S2 = Point of surface 2

			//float dist_to_S1 = distance(prev_pos,S1);
			//float dist_to_S2 = distance(prev_pos,S2);
			
			
			glm::vec3 newPos1, newNorm1, newPos2, newNorm2;
			if (!glm::intersectLineSphere<glm::vec3>(prev_pos, next_pos, *position, *radius, newPos1, newNorm1, newPos2, newNorm2)) {
				std::cout << "Couldn't compute line - sphere collision" << std::endl;
			}

			float dist_to_S1 = distance(prev_pos, newPos1);
			float dist_to_S2 = distance(prev_pos, newPos2);
			//std::cout << "Punts Raw: (" << S1.x << " " << S1.y<< " " << S1.z << ") : (" << S2.x << " " << S2.y << " " << S2.z <<")"<< std::endl;
			//std::cout << "Punts Pro: (" << newPos1.x << " " << newPos1.y << " " << newPos1.z << ") : (" << newPos2.x << " " << newPos2.y << " " << newPos2.z <<")"<< std::endl;

			if (dist_to_S1 < dist_to_S2) {
				//S1 és el punt de col·lisio que ens interessa
				collisionPoint = newPos1;
			}
			else if (dist_to_S1 > dist_to_S2) {
				//S2 és el punt de col·lisio que ens interessa
				collisionPoint = newPos2;
			}
			else {
				//impossible
			}
			return true;
		}
		return false;
	}
	void getPlane(glm::vec3& normal, float& d) override {
		normal = collisionPoint - *position;
		normal = glm::normalize(normal);
		d = glm::dot(-normal, collisionPoint);
	}
};
struct CapsuleCol : Collider {
	//...
	glm::vec3 *posA, *posB, collisionPoint;
	float* radius;

	CapsuleCol(glm::vec3 *_posA, glm::vec3 *_posB, float *_rad) {
		posA = _posA;
		posB = _posB;
		radius = _rad;
	}
	bool checkCollision(const glm::vec3& prev_pos, const glm::vec3& next_pos) override {
		glm::vec3 colPos;
		float a;

		glm::vec3 ab = (*posA - *posB);
		float abMag = Magnitude(*posA, *posB);

		a = glm::clamp(((glm::dot((next_pos - *posB), ab) / abMag) / abMag), 0.f, 1.f);
		colPos = *posB + (posA - posB) * a;

		float distance = Magnitude(next_pos, colPos) - *radius;

		if (distance <= 0 && CAPSULE_COLLISION) {
			collisionPoint = next_pos;
			return true;
		}
		return false;
	}
	void getPlane(glm::vec3& normal, float& d) override {

	}
};

glm::vec3 springforce(const glm::vec3& P1, const glm::vec3& V1, const glm::vec3& P2, const glm::vec3& V2, float L0, float ke, float kd) {
	return -(ke*(glm::distance(P1, P2) - L0) + kd * glm::dot((V1 - V2), (P1 - P2 / glm::distance(P1, P2)))) * (P1 - P2 / glm::distance(P1, P2));
}

glm::vec3 computeForces(FiberStraw& fiber, int idx, const std::vector<ForceActuator*>& force_acts) {
	glm::vec3 forces;
	for (int i = 0; i < force_acts.size(); i++) {
		forces += force_acts[i]->computeForce(FIBER_PARTICLE_MASS, fiber.positions[idx]);

	}

	// Spring Forces
	if (idx - 1 >= 0) {
		forces += springforce(fiber.positions[idx], fiber.velocities[idx], fiber.positions[idx - 1], fiber.velocities[idx - 1], fiber.distance, FIBER_ELASTICITY, FIBER_DAMPLING);
		if (idx - 2 >= 0) {
			forces += springforce(fiber.positions[idx], fiber.velocities[idx], fiber.positions[idx - 2], fiber.velocities[idx - 2], fiber.distance*2, FIBER_ELASTICITY, FIBER_DAMPLING);
		}
	}
	if (idx + 1 <= fiber.GetCount() - 1) {
		forces += springforce(fiber.positions[idx], fiber.velocities[idx], fiber.positions[idx + 1], fiber.velocities[idx + 1], fiber.distance, FIBER_ELASTICITY, FIBER_DAMPLING);
		if (idx + 2 <= fiber.GetCount() - 1) {
			forces += springforce(fiber.positions[idx], fiber.velocities[idx], fiber.positions[idx + 2], fiber.velocities[idx + 2], fiber.distance*2, FIBER_ELASTICITY, FIBER_DAMPLING);
		}
	}
	return forces;
}



void verlet(float dt, FiberStraw& fiber, const std::vector<Collider*>& colliders, const std::vector<ForceActuator*>& force_acts) {
	for (int i = 1; i < fiber.GetCount(); i++) {
		glm::vec3 forces = computeForces(fiber, i, force_acts);

		glm::vec3 newPos = fiber.positions[i] + (fiber.positions[i] - fiber.prevPositions[i]) + forces / FIBER_PARTICLE_MASS * dt*dt;
		glm::vec3 newVel = (newPos - fiber.positions[i]) / dt;

		// COLLISIONS
		for (int j = 0; j < colliders.size(); j++) {
			colliders[j]->computeCollision(fiber.positions[i], newPos);
		}

		fiber.positions[i] = newPos;
		fiber.velocities[i] = newVel;
	}

	
}

void euler(float dt, ParticleSystem& particles, const std::vector<Collider*>& colliders, const std::vector<ForceActuator*>& force_acts) {
	for (int i = 0; i < MAX_PARTICLES; i++) {
		glm::vec3 forces;
		for (int j = 0; j < force_acts.size(); j++) {
			forces += force_acts[j]->computeForce(PARTICLE_MASS, particles.particlePositions[i]);
		}
		glm::vec3 accel = glm::vec3(forces.x / PARTICLE_MASS, forces.y / PARTICLE_MASS, forces.z / PARTICLE_MASS);
		
		glm::vec3 particle_new_velocity = glm::vec3(particles.particleVelocities[i] + (dt * accel));
		glm::vec3 particle_new_position = glm::vec3(particles.particlePositions[i] + (dt * particles.particleVelocities[i]));


		for (int j = 0; j < colliders.size(); j++) {
			colliders[j]->computeCollision(particles.particlePositions[i], particles.particleVelocities[i], particle_new_position, particle_new_velocity);
		}

		particles.particlePositions[i] = particle_new_position;
		particles.particleVelocities[i] = particle_new_velocity;

	}
}

ParticleSystem* ps = new ParticleSystem();
std::vector<ForceActuator*> forces;
std::vector<Collider*> colliders;

void PhysicsInit() {
	// Do your initialization code here...
	for (int i = 0; i < MAX_PARTICLES; i++) {
		float f = Tools::Random();
		glm::vec3 newPos(
			Tools::Map(Tools::Random(), 0, 1, -5, 5), 
			Tools::Map(Tools::Random(), 0, 1, 5, 10), 
			Tools::Map(Tools::Random(), 0, 1, -5, 5));

		glm::vec3 newVel(
			-5 + Tools::Random() * 10,
			-5 + Tools::Random() * 10,
			-5 + Tools::Random() * 10);
		ps->SetParticle(i, newPos, newVel);
	}

	for (int i = 0; i < fs.FibersCount(); i++) {
		glm::vec3 newPos(
			Tools::Map(Tools::Random(), 0, 1, -5, 5),
			0,
			Tools::Map(Tools::Random(), 0, 1, -5, 5));
		
		fs.SetFiber(i, FiberStraw(newPos, Fiber::numVerts, .3f));
	}

	forces.push_back(new GravityForce());
	forces.push_back(new PositionalGravityForce(&SPHERE_MASS, &SPHERE_POS));

	colliders.push_back(new PlaneCol(glm::vec3(0, 0, 0), glm::vec3(0, -1, 0)));
	colliders.push_back(new PlaneCol(glm::vec3(0, 10, 0), glm::vec3(0, 1, 0)));
	colliders.push_back(new PlaneCol(glm::vec3(-5, 0, 0), glm::vec3(-1, 0, 0)));
	colliders.push_back(new PlaneCol(glm::vec3(5, 0, 0), glm::vec3(1, 0, 0)));
	colliders.push_back(new PlaneCol(glm::vec3(0, 0, -5), glm::vec3(0, 0, -1)));
	colliders.push_back(new PlaneCol(glm::vec3(0, 0, 5), glm::vec3(0, 0, 1)));
	colliders.push_back(new SphereCol(&SPHERE_POS, &SPHERE_RAD, &SPHERE_MASS));
	colliders.push_back(new CapsuleCol(&CAPSULE_POS_A, &CAPSULE_POS_B, &CAPSULE_RAD));


	// ...................................
}

void PhysicsUpdate(float dt) {
	// Do your update code here...
	if (SIMULATE_PARTICLES) {
		euler(dt * TIME_FACTOR, *ps, colliders, forces);

		Particles::updateParticles(0, MAX_PARTICLES, ps->ParticlesPtr());


		
	}

	if (SIMULATE_FIBERS) {

		for (int i = 0; i < fs.FibersCount(); i++) {
			verlet(dt * TIME_FACTOR, fs[i], colliders, forces);

		}
	}
	


	static float counter = .0f;
	counter += dt * TIME_FACTOR;
	if (counter >= 2*glm::pi<float>()) {
		counter = 0;
	}


	SPHERE_POS.x = SPHERE_ORBITAL_POINT.x - (glm::cos(counter * SPHERE_ORBITAL_SPEED) * SPHERE_ORBITAL_MAGNITUDE);
	SPHERE_POS.z = SPHERE_ORBITAL_POINT.z - (glm::sin(counter * SPHERE_ORBITAL_SPEED) * SPHERE_ORBITAL_MAGNITUDE);


	Sphere::updateSphere(SPHERE_POS, SPHERE_RAD);
	Capsule::updateCapsule(CAPSULE_POS_A, CAPSULE_POS_B, CAPSULE_RAD);
	// ...........................
}

void PhysicsRestart() {
	delete(ps);
	ps = new ParticleSystem();
	for (int i = 0; i < MAX_PARTICLES; i++) {
		float f = Tools::Random();
		glm::vec3 newPos(
			Tools::Map(Tools::Random(), 0, 1, -5, 5),
			Tools::Map(Tools::Random(), 0, 1, 5, 10),
			Tools::Map(Tools::Random(), 0, 1, -5, 5));

		glm::vec3 newVel(
			-5 + Tools::Random() * 10,
			-5 + Tools::Random() * 10,
			-5 + Tools::Random() * 10);

		ps->SetParticle(i, newPos, newVel);
	}

	fs = FiberSystem();
	for (int i = 0; i < fs.FibersCount(); i++) {
		glm::vec3 newPos(
			Tools::Map(Tools::Random(), 0, 1, -5, 5),
			0,
			Tools::Map(Tools::Random(), 0, 1, -5, 5));

		fs.SetFiber(i, FiberStraw(newPos, Fiber::numVerts, .3f));
	}
}

void PhysicsCleanup() {
	// Do your cleanup code here...
	delete(ps);
	// ............................
}

float Magnitude(glm::vec3 from, glm::vec3 to) {
	glm::vec3 x = to - from;
	return glm::sqrt(glm::pow(x.x, 2) + glm::pow(x.y, 2) + glm::pow(x.z, 2));
}