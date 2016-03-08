#include "common.h"
#include <cmath>

using namespace std;

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

double det(Vector3D& v1, Vector3D& v2, Vector3D& v3)
{
	return v1._x * (v2._y * v3._z - v2._z * v3._y) - v1._y *(v2._x * v3._z - v2._z * v3._x) + v1._z * (v2._x * v3._y - v2._y * v3._x);
}

void triPro(Vector3D& p1, Vector3D& p2, Vector3D& p3, Vector3D& n, double& mi, double &ma)
{
	n.normalize();
	double pnt1, pnt2, pnt3;
	pnt1 = dot(n, p1);
	pnt2 = dot(n, p2);
	pnt3 = dot(n, p3);
	mi = min(min(pnt1, pnt2), pnt3);
	ma = max(max(pnt1, pnt2), pnt3);
}

void boxPro(Vector3D& p1, Vector3D& p2, Vector3D& n, double& mi, double& ma)
{
	n.normalize();
	double x[2], y[2], z[2];
	mi = 1e6;
	ma = -1e6;
	x[0] = p1._x;
	x[1] = p2._x;
	y[0] = p1._y;
	y[1] = p2._y;
	z[0] = p1._z;
	z[1] = p2._z;
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			for (int k = 0; k < 2; k++)
			{
				double m;
				m = x[i] * n._x + y[j] * n._y + z[k] * n._z;
				ma = max(ma, m);
				mi = min(mi, m);
			}
		}
	}
}

bool check(double amax, double amin, double bmax, double bmin)
{
	if (bmin > amax) return true;
	if (amin > bmax) return true;
	return false;
}

double Vector3D::length()
{
	return sqrt(sqr(_x) + sqr(_y) + sqr(_z));
}
