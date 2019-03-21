#pragma once

#define MAX_JOINTS_PER_FIBBER 10
#define MAX_FIBERSTRAWS 100

struct FiberStraw {
public:
	FiberStraw() = default;
	FiberStraw(glm::vec3 pos, int _fibers, float _dist) {
		jointsCount = glm::clamp(_fibers, 1, MAX_JOINTS_PER_FIBBER);
		distance = _dist;

		for (int i = 0; i < MAX_JOINTS_PER_FIBBER; i++) {
			positions[i] = glm::vec3(pos.x, pos.y + (distance*i), pos.z);
		}
	}
	float GetCount() {
		return jointsCount;
	}
	float* DataPtr() {
		return &positions[0].x;
	}
	glm::vec3 positions[MAX_JOINTS_PER_FIBBER];
	//glm::vec3 velocities[MAX_JOINTS_PER_FIBBER];
private:
	int jointsCount;
	float distance;
};

struct FiberSystem {
public:
	FiberSystem(int _fiberStrawsCount = MAX_FIBERSTRAWS){
		fiberStrawsCount = glm::clamp(_fiberStrawsCount, 1, MAX_FIBERSTRAWS);
	}
	~FiberSystem(){}
	inline float* FibersPtr() {
		return &fiberStraws[0].positions[0].x;
	}
	void SetFiber(int index, FiberStraw fiber) {
		fiberStraws[index] = fiber;
	}
	int FibersCount() {
		return fiberStrawsCount;
	}

	FiberStraw& operator[](int index) {
		return fiberStraws[index];
	}

private:
	int fiberStrawsCount;
	FiberStraw fiberStraws[MAX_FIBERSTRAWS];
};