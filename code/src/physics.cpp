#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\gtc\matrix_transform.hpp>
#include <vector>

#include "Tools.h"

#define SPHERES_COUNT 3
#define CAGE_SIZE 10
#define CAGE_PLANES_COUNT 6
#define SPHERE_MASS 1
#define SPHERE_RADIUS 1

//GUI
//Sphere 1
#define SPHERE_1_MASS_MIN 1
#define SPHERE_1_MASS_MAX 100
#define SPHERE_1_RADIUS_MIN 1
#define SPHERE_1_RADIUS_MAX 10
//Sphere 2
#define SPHERE_2_MASS_MIN 1
#define SPHERE_2_MASS_MAX 100
#define SPHERE_2_RADIUS_MIN 1
#define SPHERE_2_RADIUS_MAX 10
//SPhere 3
#define SPHERE_3_MASS_MIN 1
#define SPHERE_3_MASS_MAX 100
#define SPHERE_3_RADIUS_MIN 1
#define SPHERE_3_RADIUS_MAX 10

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
	virtual bool checkCollision(const glm::vec3& next_pos, float radius) = 0;
};

struct PlaneCol : Collider {
	glm::vec3 position, normal;

	PlaneCol(){}
	PlaneCol(glm::vec3 _pos, glm::vec3 _norm) : position(_pos), normal(_norm){}

	bool checkCollision(const glm::vec3& next_pos, float radius) override {
		//...
		return false;
	}
};

struct RigidSphere : Collider {
	//...
	float mass, radius;
	glm::vec3 position;
	glm::mat3 IBody() { return glm::mat3((mass * (radius*radius)) * (2/5)); }

	RigidSphere(){}
	RigidSphere(glm::vec3 _pos, float _mass, float _rad) : mass(_mass), radius(_rad), position(_pos){}
	bool checkCollision(const glm::vec3& next_pos, float radius) override {
		//...
		return false;
	}
};

void euler(float dt, RigidSphere& sph) {
	glm::vec3 newPos;
}

float computeImpulseCorrection(float massA, glm::vec3 ra, glm::mat3 invIa, float massB, glm::vec3 rb, glm::mat3 invIb, float vrel, float epsilon, glm::vec3 normal) {
	return 0;
}

void updateColliders(Collider* A, Collider* B) {

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
		Sphere::drawSphere();
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

// BOX
// X -5 / 5
// Y 0 / 10
// Z -5 / 5
PlaneCol cage[CAGE_PLANES_COUNT];
RigidSphere spheres[SPHERES_COUNT];


void GUI() {
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);

	// Do your GUI code here....
	{	
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);//FrameRate
		
		//Gravity and Elasticity 
		//ImGui::Text("Gravity");
		//ImGui::DragFloat("Gravity Value", &GRAVITY_FORCE, 0.1f, minGrabAccel, maxGrabAccel, "%.3f");
		//ImGui::Text("Elasticity");
		//ImGui::DragFloat("Elasticity", &BOUNCE_ELASTICITY, 0.05f, 0.0f, 1.0f, "%.3f");
		//Sphere 1
		//ImGui::Text("Sphere 1");
		//ImGui::DragFloat("X 1", &CAPSULE_POS_A.x, 0.1f, SPHERE_1_MASS_MIN, SPHERE_1_MASS_MAX, "%.3f");
		//ImGui::DragFloat("X 1", &CAPSULE_POS_A.x, 0.1f, SPHERE_1_RADIUS_MIN, SPHERE_1_RADIUS_MAX, "%.3f");
		//Sphere 2
		//ImGui::Text("Sphere 2");
		//ImGui::DragFloat("X 1", &CAPSULE_POS_A.x, 0.1f, SPHERE_2_MASS_MIN, SPHERE_2_MASS_MAX, "%.3f");
		//ImGui::DragFloat("X 1", &CAPSULE_POS_A.x, 0.1f, SPHERE_2_RADIUS_MIN, SPHERE_2_RADIUS_MAX, "%.3f");
		//Sphere 3
		//ImGui::Text("Sphere 3");
		//ImGui::DragFloat("X 1", &CAPSULE_POS_A.x, 0.1f, SPHERE_3_MASS_MIN, SPHERE_3_MASS_MAX, "%.3f");
		//ImGui::DragFloat("X 1", &CAPSULE_POS_A.x, 0.1f, SPHERE_3_RADIUS_MIN, SPHERE_3_RADIUS_MAX, "%.3f");
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



void PhysicsInit() {
	// Do your initialization code here...
	cage[0] = PlaneCol(glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
	cage[1] = PlaneCol(glm::vec3(0, CAGE_SIZE, 0), glm::vec3(0, -1, 0));
	cage[2] = PlaneCol(glm::vec3(-CAGE_SIZE / 2, 0, 0), glm::vec3(1, 0, 0));
	cage[3] = PlaneCol(glm::vec3(CAGE_SIZE / 2, 0, 0), glm::vec3(-1, 0, 0));
	cage[4] = PlaneCol(glm::vec3(0, 0, -CAGE_SIZE / 2), glm::vec3(0, 0, 1));
	cage[5] = PlaneCol(glm::vec3(0, 0, CAGE_SIZE / 2), glm::vec3(0, 0, -1));



	for (int i = 0; i < SPHERES_COUNT; i++) {
		spheres[i] = RigidSphere(glm::vec3((CAGE_SIZE / 2) - (Tools::Random() * (CAGE_SIZE / 2)), 
											Tools::Random() * CAGE_SIZE,
											(CAGE_SIZE / 2) - (Tools::Random() * (CAGE_SIZE / 2))),
											SPHERE_MASS, SPHERE_RADIUS);
	}
	// ...................................
}

void PhysicsUpdate(float dt) {
	// Do your update code here...
	// ...........................
}

void PhysicsCleanup() {
	// Do your cleanup code here...
	// ............................
}