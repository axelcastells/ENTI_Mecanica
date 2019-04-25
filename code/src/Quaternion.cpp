#include "Quaternion.h"
#include "VMath.h"

using namespace Vortex;

Vortex::Quaternion::Quaternion()
{
	w = 1; x = 0; y = 0; z = 0;
}

Vortex::Quaternion::Quaternion(float _w, float _x, float _y, float _z)
{
	w = _w; x = _x; y = _y; z = _z;
}

Vortex::Quaternion::~Quaternion()
{
}

Quaternion Vortex::Quaternion::Normalized()
{
	return Normalize(*this);
}

Quaternion Vortex::Quaternion::Normalize(Quaternion _q)
{
	float magnitude = (float)Math::Sqrt(Math::Pow(_q.w, 2) + Math::Pow(_q.x, 2) + Math::Pow(_q.y, 2) + Math::Pow(_q.z, 2));
	_q.w /= magnitude;
	_q.x /= magnitude;
	_q.y /= magnitude;
	_q.z /= magnitude;
	return _q;
}

Quaternion Vortex::Quaternion::operator+(Quaternion _q)
{
	Quaternion _res = Quaternion();
	_res.w = this->w + _q.w;
	_res.x = this->w + _q.w;
	_res.y = this->y + _q.y;
	_res.z = this->z + _q.z;
	_res = Normalize(_res);
	return _res;
}

Quaternion Vortex::Quaternion::operator*(Quaternion _q)
{
	Quaternion _res = Quaternion();
	_res.w = (this->w * _q.w) - (this->x * _q.x) - (this->y * _q.y) - (this->z * _q.z);
	_res.x = (this->w * _q.x) + (this->x * _q.w) + (this->y * _q.z) - (this->z * _q.y);
	_res.y = (this->w * _q.y) + (this->y * _q.w) - (this->x * _q.z) + (this->z * _q.x);
	_res.z = (this->w * _q.z) + (this->z * _q.w) + (this->x * _q.y) - (this->y * _q.x);
	return Normalize(_res);
}

Quaternion Vortex::Quaternion::Inverse(Quaternion _q)
{
	_q = Normalize(_q);
	_q.x *= -1;
	_q.y *= -1;
	_q.z *= -1;

	return _q;
}

Quaternion Vortex::Quaternion::AxisAngleToQuaternion(Vector3 _axis, float _angle)
{
	_axis = _axis.Normalized();
	Quaternion _q;
	_q.x = (float)(_axis.x * Math::Sin(_angle / 2)),
	_q.y = (float)(_axis.y * Math::Sin(_angle / 2)),
	_q.z = (float)(_axis.z * Math::Sin(_angle / 2)),
	_q.w = (float)(Math::Cos(_angle / 2));

	return _q;
}

AxisAngle Vortex::Quaternion::QuaternionToAxisAngle(Quaternion _q)
{
	AxisAngle _axAng;
	_axAng.angle = (float)(2 * Math::Acos(_q.w));
	_axAng.x = (float)(_q.x / Math::Sqrt(1 - _q.w * _q.w));
	_axAng.y = (float)(_q.y / Math::Sqrt(1 - _q.w * _q.w));
	_axAng.z = (float)(_q.z / Math::Sqrt(1 - _q.w * _q.w));
	return _axAng;
}

Quaternion Vortex::Quaternion::Derivate(float dt, Vector3 angularSpeed)
{
	angularSpeed = angularSpeed / 2;
	return Quaternion(0,angularSpeed.x, angularSpeed.y, angularSpeed.z) * *this;
}
