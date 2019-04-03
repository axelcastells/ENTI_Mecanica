#pragma once
#include "Vector.h"


namespace Vortex {
	namespace Math {
		const float DEG2RAD = 0.0174532924;
		const float RAD2DEG = 57.29578;

		double Sqrt(double val);
		double Pow(double val, double exponent);
		double Clamp(double value, double min, double max);

		double Sin(double _angle);
		double Cos(double _angle);
		double Acos(double _angle);

		Vector3 Translate(Vector3 _vec, Vector3 _trans);
		Vector3 Rotate(Vector3 _vec, Vector3 _axis, float _angle);
		Vector3 Scale(Vector3 _vec, Vector3 _scale);


	}
}