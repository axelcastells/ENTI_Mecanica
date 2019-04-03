#pragma once

namespace Vortex {
	
	class Vector3 {
	public:
		float x, y, z;

#pragma region Constructors & Destructor
		Vector3();
		Vector3(float _xyz);
		Vector3(float _x, float _y, float _z);
		~Vector3();
#pragma endregion

#pragma region Operators
		Vector3 operator+(Vector3 v);
		Vector3 operator-(Vector3 v);
		Vector3 operator*(Vector3 v);
		Vector3 operator/(Vector3 v);

		Vector3 operator+(float f);
		Vector3 operator*(float f);
#pragma endregion

		inline static Vector3 Zero() { return Vector3(0); }
		inline static Vector3 One() { return Vector3(1); }

		static Vector3 Normalize(Vector3 v);
		void Normalize();
		Vector3 Normalized();

		static Vector3 Cross(Vector3 l, Vector3 r);
		static double Dot(Vector3 l, Vector3 r);

		static float Magnitude(Vector3 v);
		float Magnitude();

		float Distance(Vector3 v1, Vector3 v2);
	};
}