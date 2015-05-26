#include "common.h"

double sqr(double x)
{
	return x * x;
}

double dot(Vector3D& v1, Vector3D& v2)
{
	return (v1._x * v2._x + v1._y * v2._y + v1._z * v2._z);
}

void Vector3D::normalize()
{
	double len = sqrt(sqr(_x) + sqr(_y) + sqr(_z));
	_x = _x / len;
	_y = _y / len;
	_z = _z / len;
}
