#include "Vector.h"
#include "VMath.h"

using namespace Vortex;

Vector3::Vector3() {
	x = y = z = 0;
}

Vector3::Vector3(float _xyz) {
	x = y = z = _xyz;
}

Vector3::Vector3(float _x, float _y, float _z) {
	x = _x;
	y = _y;
	z = _z;
}
Vector3::~Vector3() {
	
}

void Vortex::Vector3::Normalize()
{
	*this = Normalize(*this);
}

Vector3 Vortex::Vector3::Normalized()
{
	Vector3 n = Normalize(*this);
	return n;
}

Vector3 Vortex::Vector3::Normalize(Vector3 v)
{
	float mod = (float)Math::Sqrt(Math::Pow(v.x, 2) + Math::Pow(v.y, 2) + Math::Pow(v.z, 2));
	v.x /= mod;
	v.y /= mod;
	v.z /= mod;
	return v;
}

Vector3 Vortex::Vector3::operator+(Vector3 v)
{
	return Vector3(x + v.x, y + v.y, z + v.z);
}

Vector3 Vortex::Vector3::operator-(Vector3 v)
{
	return Vector3(x - v.x, y - v.y, z - v.z);
}

Vector3 Vortex::Vector3::operator*(Vector3 v)
{
	return Vector3(x * v.x, y * v.y, z * v.z);
}

Vector3 Vortex::Vector3::operator/(Vector3 v)
{
	return Vector3(x / v.x, y / v.y, z / v.z);
}

Vector3 Vortex::Vector3::operator+(float f)
{
	return Vector3(x + f, y + f, z + f);
}

Vector3 Vortex::Vector3::operator*(float f)
{
	return Vector3(x * f, y * f, z * f);
}

Vector3 Vortex::Vector3::Cross(Vector3 l, Vector3 r)
{
	return Vector3(	l.y * r.z - l.z * r.y,
					l.z * r.x - l.x * r.z,
					l.x * r.y - l.y * r.x);
}

double Vortex::Vector3::Dot(Vector3 l, Vector3 r)
{
	return ((l.x * r.x) + (l.y * r.y) + (l.z * r.z));
}

float Vortex::Vector3::Magnitude(Vector3 v)
{
	return (float)Math::Sqrt(Math::Pow(v.x, 2) + Math::Pow(v.y, 2) + Math::Pow(v.z, 2));
}

float Vortex::Vector3::Magnitude()
{
	return Magnitude(*this);
}

float Vortex::Vector3::Distance(Vector3 v1, Vector3 v2)
{
	return Magnitude(v1 - v2);
}
