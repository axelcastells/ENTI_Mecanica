#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\gtc\matrix_transform.hpp>
#include <vector>
#include <thread>
#include <iostream>
#include <string>

#define DEG2RAD 0.0174532924
#define RAD2DEG 57.29578

#define SPHERE_DRAGC 0.47f

#define SPHERE_IBODY_MATRIX(m, r) glm::mat3((m * (r*r)) * (2.f/5.f))

#define SPHERES_COUNT 1
#define CAGE_SIZE 10
#define CAGE_PLANES_COUNT 6
#define SPHERE_START_MASS 3
#define SPHERE_START_RADIUS 1

#define MESH_DISTANCE_POINTS 0.75
//GUI
#define MESH_OFFSET_X -5
#define MESH_OFFSET_Z -5
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
static float SPHERE_MASS = 1;
static float SPHERE_RADIUS = 1;
static float GRAVITY_FORCE = 9.81f;					// Gravity Force
static glm::vec3 GRAVITY_VECTOR = { 0, -1, 0 };		// Gravity Vector
static float RESTITUTION_FACTOR = 1.f;				// Elasticity
static bool SIMULATE = true;						// Simulation Boolean
static float TIME_SCALE = 1.f;						// Time Scale
static float CURRENT_TIME = 0.f;

//static float FREQUENCY = 1.f;
//static float AMPLITUDE = 1.f;

static unsigned int WAVES_COUNT = 3;
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


struct BuoyantSphere {
	glm::vec3 centerSphere;
	glm::vec3 sphereVelocity;
	glm::vec3 sphereForce;
	float yPoints;
	int countPoints;
	void Init() {
		SPHERE_RADIUS = (rand() % 1) + 1;
		centerSphere.x = (rand() % 2) - 1;
		centerSphere.y = (rand() % 3) + 2;
		centerSphere.z = (rand() % 2) - 1;
	}
	bool DistanceToPoint(glm::vec3 point)
	{
		float distanciaPla = (centerSphere.y - SPHERE_RADIUS) - point.y;
		return distanciaPla <= 0;
	}
};

struct Wave {
	Wave() {
		frequence = (rand() % 2) + 0.1f;
		amplitude = (rand() % 2) + 0.1f;
		kBlood = glm::vec3((rand() % 2) - 1, (rand() % 2) - 1, (rand() % 2) - 1);
		y = glm::vec3(0.0f, 1.0f, 0.0f);
		density = 1;
		vSub = 1;
		kItalic = kBlood.length();
	}

	glm::vec3 kBlood;
	glm::vec3 y;

	float frequence;
	float amplitude;

	float kItalic;
	float density;
	float vSub;
};

struct FluidSystem {
	std::vector<Wave> waves;
	inline Wave& operator[](int i) {
		return waves[i];
	}
	inline unsigned int size() {
		return waves.size();
	}
};

FluidSystem flsys;
BuoyantSphere sph;

glm::vec3 getInitPos(int i, int j, float initY = 3.f) {
	return glm::vec3(MESH_OFFSET_X+(i*MESH_DISTANCE_POINTS), initY, MESH_OFFSET_Z+(j*MESH_DISTANCE_POINTS));
}
glm::vec3 getGerstnerPos(FluidSystem* FLSys, glm::vec3 position, float accum_time = 0.f) {
	FluidSystem& FS = *FLSys;
	
	glm::vec3 gerstnerPos;
	for (int k = 0; k < FS.size(); k++) {
		glm::vec3 v = (FS[k].kBlood / FS[k].kItalic)*FS[k].amplitude*sin(dot(FS[k].kBlood, position) - FS[k].frequence * accum_time);

		gerstnerPos.x = position.x - v.x;// FLSys->initPos[i].x - v.x;
		gerstnerPos.z = position.z - v.z;//FLSys->initPos[i].z - v.z;
		gerstnerPos.y += (glm::vec3(FS[k].amplitude*cos(dot(FS[k].kBlood, position) - FS[k].frequence * accum_time))).y;
	}

	return gerstnerPos;
}
glm::vec3 getGerstnerPos(FluidSystem* FLSys, int i, int j,  float accum_time = 0.f) {
	FluidSystem& FS = *FLSys;
	
	glm::vec3 gerstnerPos;
	for (int k = 0; k < FS.size(); k++) {
		glm::vec3 v = (FS[k].kBlood / FS[k].kItalic)*FS[k].amplitude*sin(dot(FS[k].kBlood, getInitPos(i,j)) - FS[k].frequence * accum_time);

		gerstnerPos.x = getInitPos(i, j).x - v.x;// FLSys->initPos[i].x - v.x;
		gerstnerPos.z = getInitPos(i, j).z - v.z;//FLSys->initPos[i].z - v.z;
		gerstnerPos.y += (glm::vec3(FS[k].amplitude*cos(dot(FS[k].kBlood, getInitPos(i,j)) - FS[k].frequence * accum_time))).y;
	}

	return gerstnerPos;
}

glm::vec3 computeBuoyancyForce(FluidSystem* FLSys, BuoyantSphere* BSph, float accum_time) {
	FluidSystem& FS = *FLSys;
	glm::vec3 buoyancy(0);
	for (int i = 0; i < FS.size(); i++) {
		if (BSph->countPoints > 0) {
			float d = (BSph->yPoints / BSph->countPoints) - (BSph->centerSphere.y - SPHERE_RADIUS);
			FS[i].vSub = d * SPHERE_RADIUS*SPHERE_RADIUS;
			buoyancy += FS[i].density * GRAVITY_FORCE * FS[i].vSub*FS[i].y;

		}

	}
	
	return buoyancy;
}

glm::vec3 computeDragForce(FluidSystem* FLSys, BuoyantSphere* BSph, float accum_time) {
	FluidSystem& FS = *FLSys;
	glm::vec3 drag(0);
	for (int i = 0; i < FS.size(); i++) {
		if (BSph->countPoints > 0) {

			glm::vec3 relativeVel = BSph->sphereVelocity - FS[i].kBlood;
			glm::vec3 relativeVelMod = glm::vec3(relativeVel.x * glm::length(relativeVel), relativeVel.y * glm::length(relativeVel), relativeVel.z * glm::length(relativeVel));
			
			float values = -0.5*FS[i].density*SPHERE_DRAGC * glm::abs(BSph->yPoints / BSph->countPoints);
			drag += values*relativeVelMod;
		}
	}
	return drag;
}

void updateSphere(FluidSystem* FLSys, BuoyantSphere* BSph, float accum_time, float dt) {

	glm::vec3 force = computeBuoyancyForce(&flsys, &sph, accum_time);
	glm::vec3 drag = computeDragForce(&flsys, &sph, accum_time);

	BSph->sphereForce += force + drag + GRAVITY_FORCE * GRAVITY_VECTOR*SPHERE_MASS;
	BSph->sphereVelocity = dt * BSph->sphereForce / SPHERE_MASS;
	BSph->sphereVelocity = { 0, BSph->sphereVelocity.y, 0 };
	BSph->centerSphere = BSph->centerSphere + dt * BSph->sphereVelocity;

	Sphere::updateSphere(BSph->centerSphere, SPHERE_RADIUS);
}

namespace System {

	std::vector<glm::vec3> points;

	void Init() {
		for (int i = 0; i < WAVES_COUNT; i++) {
			flsys.waves.push_back(Wave());
		}

		CURRENT_TIME = 0;
		sph.Init();
	}
	void Update(float dt) {
		CURRENT_TIME += dt;
		points.clear();

		glm::vec3 force(0), drag(0);
		sph.yPoints = 0;
		sph.countPoints = 0;

		for (int i = 0; i < Mesh::numRows; i++) {
			for (int j = 0; j < Mesh::numCols; j++) {
				glm::vec3 p = getGerstnerPos(&flsys, i, j, CURRENT_TIME);
				points.push_back(p);

				if (sph.DistanceToPoint(getGerstnerPos(&flsys, sph.centerSphere, CURRENT_TIME))) {
					sph.yPoints += getGerstnerPos(&flsys, sph.centerSphere, CURRENT_TIME).y;
					sph.countPoints++;
				}
				
			}
		}
		
		updateSphere(&flsys, &sph, CURRENT_TIME, dt);
	}
}


// Boolean variables allow to show/hide the primitives
bool renderSphere = true;
bool renderCapsule = false;
bool renderParticles = true;
bool renderMesh = true;
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

void PhysicsReset() {
	sph = BuoyantSphere();
	flsys.waves.clear();
	System::Init();
	//Old::Init();
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

void GUI() {
	{	//FrameRate
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		////TODO
		ImGui::Checkbox("Play Simulation", &SIMULATE);
		ImGui::DragFloat3("Gravity Vector", &GRAVITY_VECTOR.x, 0.1f, -1, 1, "%.3f");

		ImGui::Text("Sphere");
		ImGui::DragFloat("Mass", &SPHERE_MASS, 0.1f, 0.0f, 30.0f, "%.1f");
		ImGui::DragFloat("Radius", &SPHERE_RADIUS, 0.1f, 0.1f, 3.0f, "%.1f");

		ImGui::Text("Using Gerstner Wave");
		ImGui::Text("Time %.1f", CURRENT_TIME);
		for (int i = 0; i < WAVES_COUNT; i++) {
			ImGui::Text("Wave: %i", i);

			ImGui::DragFloat3(std::string("Direction: " + std::to_string(i)).c_str(), &flsys[i].kBlood.x, 0.1f, 0.0f, 0.5f, "%.1f");
			ImGui::DragFloat(std::string("Frequence: " + std::to_string(i)).c_str(), &flsys[i].frequence, 0.1f, 1.0f, 10.0f, "%.1f");
			ImGui::DragFloat(std::string("Amplitude: " + std::to_string(i)).c_str(), &flsys[i].amplitude, 0.1f, 1.0f, 10.0f, "%.1f");
			ImGui::DragFloat(std::string("Density: " + std::to_string(i)).c_str(), &flsys[i].density, 0.1f, 15.0f, 20.0f, "%.1f");
			
		}
		

		if (ImGui::Button("Reset", ImVec2(50, 20)))
		{
			PhysicsReset();
		}
	}

	// Example code -- ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	bool show_test_window = false;
	if (show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}



void PhysicsUpdate(float dt) {
	if (SIMULATE) {

		//dt *= TIME_SCALE;
		//// Do your update code here...

		//counter += dt;
		//if (counter >= 15) {
		//	counter = 0;
		//	ResetSimulation();
		//}

		System::Update(dt);
		Mesh::updateMesh(&System::points.data()->x);
		//Old::Update(dt);
		//Mesh::updateMesh(Old::ParticlesToFloatPointer());
		// ...........................
	}
}

void PhysicsCleanup() {
	// Do your cleanup code here...
	// ............................
}