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

#define SPHERE_IBODY_MATRIX(m, r) glm::mat3((m * (r*r)) * (2.f/5.f))

#define SPHERES_COUNT 3
#define CAGE_SIZE 10
#define CAGE_PLANES_COUNT 6
#define SPHERE_START_MASS 3
#define SPHERE_START_RADIUS 1

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

namespace MeshParticles
{
	//Mesh
	std::vector<glm::vec3> list;
	std::vector<glm::vec3> initPos;
	glm::vec3 kBlood = glm::vec3(0.0f, 0.0f, 0.5f);
	glm::vec3 y = glm::vec3(0.0f, 1.0f, 0.0f);

	float time, t, w, A, kItalic, density, gravity, vSub;
	float initalDistancePoints = 0.75;

	//Sphere
	glm::vec3 centerSphere;
	glm::vec3 sphereVelocity;
	glm::vec3 sphereForce;
	float r = 1.0f;
	float m = 20.0f;

	float* ParticlesToFloatPointer()
	{
		float* v = new float[list.size() * 3];
		for (int i = 0; i < list.size(); i++)
		{
			v[i * 3 + 0] = list[i].x;
			v[i * 3 + 1] = list[i].y;
			v[i * 3 + 2] = list[i].z;
		}
		return v;
	}
	void ResetValues()
	{
		//Sumatori del temps de simulacio
		t = 0.0f;

		//Freq�encia. Mantenim els valors de la W
		if (w < 1.0f)w = 5.0f;

		//Amplitud de la onada
		A = 0.5f;

		//La longitud del vector
		kItalic = kBlood.length();

		//Gravetat
		gravity = -9.8f;

		//DensitatFluit
		density = 15.0f;

		//Volum Desplasat
		vSub = 0.0f;

		//Mesh
		list.clear();
		initPos.clear();
		float x;
		float z = -5.5f;
		float y = 4.0f;

		for (int j = 0; j < Mesh::numRows; j++)
		{
			z += initalDistancePoints;
			x = -5.5f;
			for (int i = 0; i < Mesh::numCols; i++)
			{
				x += initalDistancePoints;
				glm::vec3 p(x, y, z);
				list.push_back(p);
				initPos.push_back(p);
			}
		}
		Mesh::updateMesh(ParticlesToFloatPointer());

		//Sphere
		r = (rand() % 1) + 1;
		centerSphere.x = (rand() % 2) - 1;
		centerSphere.y = (rand() % 3) + 2;
		centerSphere.z = (rand() % 2) - 1;
		Sphere::updateSphere(centerSphere, r);
		sphereForce.y = -9.8f;

	}

	//Extret de la practica de rebot de les particules
	bool DistancePointToPoint(glm::vec3 point)
	{
		float distanciaPla = (centerSphere.y - r) - point.y;
		return distanciaPla <= 0;
	}
	void Update(float dt)
	{
		//Temps de simulacio
		time += dt;
		if (time >= 20)
		{
			ResetValues();
			time = 0;
		}
		else
		{
			//Sumatori de la simulacio
			t += dt;
			float yPoints = 0.0f;
			int countPoints = 0;
			glm::vec3 bouyancy;

			for (int i = 0; i < list.size(); i++)
			{
				glm::vec3 v = (kBlood / kItalic)*A*sin(dot(kBlood, initPos[i]) - w * t);
				list[i].x = initPos[i].x - v.x;
				list[i].z = initPos[i].z - v.z;

				list[i].y = (glm::vec3(A*cos(dot(kBlood, initPos[i]) - w * t))).y;
				if (DistancePointToPoint(list[i]))
				{
					yPoints += list[i].y;
					countPoints++;
				}
			}

			//SPHERE
			//Moure esfera Euler SemiImplicit
			if (countPoints > 0)
			{
				//centerSphere.y = (yPoints / countPoints);
				float d = (yPoints / countPoints) - (centerSphere.y - r);
				vSub = d * r*r;
				bouyancy = density * -gravity * vSub*y;

			}
			sphereForce = bouyancy + glm::vec3(0.0f, gravity, 0.0f)*m;
			sphereVelocity += dt * sphereForce / m;
			centerSphere = centerSphere + dt * sphereVelocity;

			//Actualitzar
			Sphere::updateSphere(centerSphere, r);
		}
	}
};


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



void GUI() {
	{	//FrameRate
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		//TODO
		ImGui::Checkbox("Play Simulation", &SIMULATE);
		ImGui::Text("Using Gerstner Wave");
		ImGui::Text("Time %.1f", MeshParticles::time);
		ImGui::DragFloat("Frequence", &MeshParticles::w, 0.1f, 1.0f, 10.0f, "%.1f");
		ImGui::DragFloat("Density", &MeshParticles::density, 0.1f, 15.0f, 20.0f, "%.1f");
		ImGui::DragFloat("Mass", &MeshParticles::m, 0.1f, 20.0f, 30.0f, "%.1f");

		ImGui::DragFloat("Directoin Waves X", &MeshParticles::kBlood.x, 0.1f, 0.0f, 0.5f, "%.1f");

		ImGui::DragFloat("Directoin Waves Z", &MeshParticles::kBlood.z, 0.1f, 0.0f, 0.5f, "%.1f");

		if (ImGui::Button("Reset", ImVec2(50, 20)))
		{
			MeshParticles::ResetValues();
		}
	}

	// Example code -- ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	bool show_test_window = false;
	if (show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

void PhysicsReset() {
	MeshParticles::ResetValues();
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

		MeshParticles::Update(dt);
		Mesh::updateMesh(MeshParticles::ParticlesToFloatPointer());
		// ...........................
	}
}

void PhysicsCleanup() {
	// Do your cleanup code here...
	// ............................
}