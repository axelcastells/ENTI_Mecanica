#pragma once
#include "Vector.h"

using namespace Vortex;

class AxisAngle
{
	// Properties:
public:
	float x, y, z, angle;

	// Functions:
	AxisAngle() { x = y = z = angle = 0; }
	AxisAngle(Vector3 _axis, float _angle)
	{
		x = _axis.x; y = _axis.y; z = _axis.z; angle = _angle;
	}
	AxisAngle(float _x, float _y, float _z, float _angle)
	{
		x = _x; y = _y; z = _z; angle = _angle;
	}
	~AxisAngle() { }
private:
};