#include "VMath.h"
#include <math.h>
#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtc\type_ptr.hpp>

using namespace Vortex;

double Vortex::Math::Sqrt(double val)
{
	return sqrt(val);
}

double Vortex::Math::Pow(double val, double exponent)
{
	return pow(val, exponent);
}

double Vortex::Math::Clamp(double value, double min, double max)
{
	return (value < min) ? min : (value > max) ? max : value;
}

double Vortex::Math::Sin(double _angle)
{
	
	return glm::sin(_angle);
}

double Vortex::Math::Cos(double _angle)
{
	return glm::cos(_angle);
}

double Vortex::Math::Acos(double _angle)
{
	return glm::acos(_angle);
}

Vector3 Vortex::Math::Translate(Vector3 _vec, Vector3 _trans)
{
	glm::mat4 trans = glm::mat4(1.0f);
	glm::vec4 vec(_vec.x, _vec.y, _vec.z, 1.0f);

	trans = glm::translate(trans, glm::vec3(_trans.x, _trans.y, _trans.z));
	vec = trans * vec;
	return Vector3(vec.x, vec.y, vec.z);
}

Vector3 Vortex::Math::Rotate(Vector3 _vec, Vector3 _axis, float _angle)
{
	glm::mat4 trans = glm::mat4(1.0f);
	glm::vec4 vec(_vec.x, _vec.y, _vec.z, 1.0f);

	trans = glm::rotate(trans, glm::radians(_angle), glm::vec3(0.0, 0.0, 1.0));
	vec = trans * vec;
	return Vector3(vec.x, vec.y, vec.z);
}

Vector3 Vortex::Math::Scale(Vector3 _vec, Vector3 _scale)
{
	glm::mat4 trans = glm::mat4(1.0f);
	glm::vec4 vec(_vec.x, _vec.y, _vec.z, 1.0f);

	trans = glm::scale(trans, glm::vec3(0.5, 0.5, 0.5));
	vec = trans * vec;
	return Vector3(vec.x, vec.y, vec.z);
}
