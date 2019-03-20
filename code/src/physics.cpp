#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\gtc\matrix_transform.hpp>
#include "ParticleSystem.h"
#include "FiberStraw.h"
#include "Tools.h"

#pragma region GlobalData
//Colisions
static bool SPHERE_COLLISION = true;
static bool CAPSULE_COLLISION = true;
//Velocitat i Activacio del funcionament
static bool SIMULATE_PARTICLES = false;
static bool GRAVITY_ACTIVE = true;
static bool POSITIONAL_GRAVITY_ACTIVE = true;
static float TIME_FACTOR = 1.0f;
//Masa de les particules
static float PARTICLE_MASS = 1.f;
// Factors de gravetat
static float GRAVITY_FORCE = 9.81f;
static glm::vec3 GRAVITY_VECTOR = { 0,-1,0 };
//Friccio i Elasticitat
static float BOUNCE_ELASTICITY = 0.8f;
static float FRICTION_FACTOR = .2f;
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

//defines pel GUI
#define minPmass 1.0
#define maxPmass 100.0

#define minSphmass 1.0
#define maxSphmass 100.0

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
		ImGui::Checkbox("Play/Pause the Simulation", &SIMULATE_PARTICLES);
		ImGui::DragFloat("Time Scale", &TIME_FACTOR, 0.01f, 0.1f, 1.0f, "%.3f");
		if (ImGui::Button("Reset", ImVec2(50, 20))) { //Trobar i reactivar la funcio que inicia la simulacio
			PhysicsRestart();
		}
		ImGui::DragFloat("Particles Mass", &PARTICLE_MASS, 0.05f, minPmass, maxPmass, "%.3f");

		//Elasticitat i Friccio Particules
		ImGui::Text("Elasticity & Friction");
		ImGui::DragFloat("Elasticity", &BOUNCE_ELASTICITY, 0.05f, 0.0f, 1.0f, "%.3f");
		ImGui::DragFloat("Friction", &FRICTION_FACTOR, 0.05f, 0.0f, 1.0f, "%.3f");

		ImGui::Text("Colliders");
		//Sphere
		//Use Sphere Collider
		ImGui::Text("Sphere");
		ImGui::Checkbox("Sphere Collision", &SPHERE_COLLISION);
		ImGui::DragFloat("Mass", &SPHERE_MASS, 1.0f, minSphmass, maxSphmass, "%.3f");
		ImGui::DragFloat("X", &SPHERE_POS.x, 0.1f, minSphposX, maxSphposX, "%.3f");
		ImGui::DragFloat("Y", &SPHERE_POS.y, 0.1 ,minSphposY, maxSphposY, "%.3f");
		ImGui::DragFloat("Z", &SPHERE_POS.z, 0.1f, minSphposZ, maxSphposZ, "%.3f");
		ImGui::DragFloat("Sphere Radius", &SPHERE_RAD, 0.05f, minSphrad, maxSphrad, "%.3f");
		//Capsule
		//Use Capsule Collider
		ImGui::Text("Capsule");
		ImGui::Checkbox("Capsule Collision", &CAPSULE_COLLISION);
		ImGui::DragFloat("X 1", &CAPSULE_POS_A.x, 0.1f, minCapposA_X, maxCapposA_X, "%.3f");
		ImGui::DragFloat("Y 1", &CAPSULE_POS_A.y, 0.1f, minCapposA_Y, maxCapposA_Y, "%.3f");
		ImGui::DragFloat("Z 1", &CAPSULE_POS_A.z, 0.1f, minCapposA_Z, maxCapposA_Z, "%.3f");
		ImGui::DragFloat("X 2", &CAPSULE_POS_B.x, 0.1f, minCapposB_X, maxCapposB_X, "%.3f");
		ImGui::DragFloat("Y 2", &CAPSULE_POS_B.y, 0.1f, minCapposB_Y, maxCapposB_Y, "%.3f");
		ImGui::DragFloat("Z 2", &CAPSULE_POS_B.z, 0.1f, minCapposB_Z, maxCapposB_Z, "%.3f");
		ImGui::DragFloat("Capsule Radius", &CAPSULE_RAD, 0.05f, minCaprad, maxCaprad, "%.3f");

		ImGui::Text("Forces");
		//Use Gravity
		ImGui::Checkbox("Global Gravity", &GRAVITY_ACTIVE);
		ImGui::DragFloat("Gravity Value", &GRAVITY_FORCE, 0.1f, minGrabAccel, maxGrabAccel, "%.3f");
		ImGui::Checkbox("Positional Gravity Sphere", &POSITIONAL_GRAVITY_ACTIVE);
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
	void computeCollision(const glm::vec3& old_pos, const glm::vec3& old_vel, glm::vec3& new_pos, glm::vec3& new_vel) 
	{
		if (checkCollision(old_pos, new_pos)) {
			glm::vec3 normal;
			float d;
			getPlane(normal, d);
			
			new_pos = old_pos - (1 + BOUNCE_ELASTICITY) * (glm::dot(normal, old_pos) + d) * normal;
			new_vel = old_vel - (1 + BOUNCE_ELASTICITY) * glm::dot(normal, old_vel) * normal;

			glm::vec3 nVel, tVel;
			nVel = glm::dot(normal, old_vel) * normal;
			tVel = old_vel - nVel;

			new_vel = old_vel - (FRICTION_FACTOR * tVel);
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
		glm::vec3 vector = next_pos - *position;
		float magnitude = glm::sqrt(glm::pow(vector.x, 2) + glm::pow(vector.y, 2) + glm::pow(vector.z, 2));
		float distance = glm::abs(magnitude);
		
		if (distance <= *radius && SPHERE_COLLISION) {
			collisionPoint = next_pos;
			return true;
		}
		return false;
	}
	void getPlane(glm::vec3& normal, float& d) override {
		normal = collisionPoint - *position;
		float mag = glm::sqrt(glm::pow(normal.x, 2) + glm::pow(normal.y, 2) + glm::pow(normal.z, 2));
		normal = normal / mag;
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

glm::vec3 computeForces(FiberStraw& fiber, int idx, const std::vector<ForceActuator*>& force_acts) {
	return glm::vec3();
}

glm::vec3 springforce(const glm::vec3& P1, const glm::vec3& V1, const glm::vec3& P2, const glm::vec3& V2, float L0, float ke, float kd) {
	return glm::vec3();
}

void euler(float dt, ParticleSystem& particles, const std::vector<Collider*>& colliders, const std::vector<ForceActuator*>& force_acts) {
	for (int i = 0; i < MAX_PARTICLES; i++) {
		glm::vec3 forces;
		for (int j = 0; j < force_acts.size(); j++) {
			forces += force_acts[j]->computeForce(PARTICLE_MASS, particles.particlePositions[i]);
		}
		glm::vec3 accel = glm::vec3(forces.x / PARTICLE_MASS, forces.y / PARTICLE_MASS, forces.z / PARTICLE_MASS);
		particles.particleVelocities[i] += (dt * accel);
		particles.particlePositions[i] += (dt * particles.particleVelocities[i]);

		for (int j = 0; j < colliders.size(); j++) {
			colliders[j]->computeCollision(particles.particlePositions[i], particles.particleVelocities[i], 
											particles.particlePositions[i], particles.particleVelocities[i]);
		}

	}
}

void verlet(float dt, FiberStraw& fiber, const std::vector<Collider*>& colliders, const std::vector<ForceActuator*>& force_acts) {
	
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

	static float counter = .0f;
	counter += dt;
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