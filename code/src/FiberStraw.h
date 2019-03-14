#pragma once

#define MAX_JOINTS_PER_FIBBER 3
#define MAX_FIBERSTRAWS 100

struct FiberStraw {
public:
private:
	
};

struct FiberSystem {
public:
	FiberSystem(int _fibersCount = MAX_FIBERSTRAWS){
		fibersCount = glm::clamp(_fibersCount, 1, MAX_FIBERSTRAWS);
	}
	~FiberSystem(){}

	int FibersCount() {
		return fibersCount;
	}
private:
	int fibersCount;
	FiberStraw fiberStraws[MAX_FIBERSTRAWS];
};