#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\gtc\matrix_transform.hpp>
#include "ParticleSystem.h"
#include "Tools.h"

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

// ------------------------------------------------------------------------------------------

ParticleSystem ps = ParticleSystem();

void PhysicsInit() {
	// Do your initialization code here...
	for (int i = 0; i < 5000; i++) {
		float f = Tools::Random();
		glm::vec3 newPos(
			Tools::Map(Tools::Random(), 0, 1, -5, 5), 
			Tools::Map(Tools::Random(), 0, 1, 5, 10), 
			Tools::Map(Tools::Random(), 0, 1, -5, 5));

		glm::vec3 newVel(
			Tools::Random() * 5,
			Tools::Random() * 5,
			Tools::Random() * 5);

		ps.SetParticle(i, newPos, newVel);

		ps.ParticlesPtr();
	}
	// ...................................
}

void PhysicsUpdate(float dt) {
	// Do your update code here...
	Particles::updateParticles(0, PARTICLE_COUNT, ps.ParticlesPtr());
	// ...........................
}

void PhysicsCleanup() {
	// Do your cleanup code here...
	// ............................
}