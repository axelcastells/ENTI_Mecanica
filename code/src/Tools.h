#pragma once
#include <stdlib.h>
#include <time.h>

namespace Tools {
	static float Random() {
		return ((float)rand() / (RAND_MAX));
	}
	static double Map(double val, double inMin, double inMax, double outMin, double outMax) {
		return outMin + (outMax - outMin) * ((val - inMin) / (inMax - inMin));
	}
}