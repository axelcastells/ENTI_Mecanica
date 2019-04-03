#pragma once
#include "Vector.h"
#include "AxisAngle.h"

using namespace Vortex;

namespace Vortex {
	class Quaternion
	{
		// Properties:
	public:
		Quaternion();
		Quaternion(float _w, float _x, float _y, float _z);
		~Quaternion();

		Quaternion Normalized();
		static Quaternion Normalize(Quaternion _q);

		Quaternion operator +(Quaternion _q);
		Quaternion operator *(Quaternion _q);

		static Quaternion Inverse(Quaternion _q);
		static Quaternion AxisAngleToQuaternion(Vector3 _axis, float _angle);
		static AxisAngle QuaternionToAxisAngle(Quaternion _q);

	private:
		float w, x, y, z;


		// Functions:










		



	};
}