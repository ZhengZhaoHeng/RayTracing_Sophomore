#ifndef __COMMON_H_
#define __COMMON_H_

#include <cmath>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\opencv.hpp>
#include <iostream>

#define EPS 1e-4
#define TRACEDEPTH 6

double sqr(double);

class Vector3D
{
public:
	Vector3D():_x(0), _y(0), _z(0){}
	Vector3D(double x, double y, double z):_x(x), _y(y), _z(z){}
	~Vector3D(){}
	void normalize();
	friend Vector3D operator +(const Vector3D& v1, const Vector3D& v2){ return Vector3D(v1._x + v2._x, v1._y + v2._y, v1._z + v2._z); }
	friend Vector3D operator -(const Vector3D& v1, const Vector3D& v2){ return Vector3D(v1._x - v2._x, v1._y - v2._y, v1._z - v2._z); }
	friend Vector3D operator /(const Vector3D& v, const double& rate){ return Vector3D(v._x / rate, v._y / rate, v._z / rate); }
	friend Vector3D operator *(const Vector3D& v, const double& rate){ return Vector3D(v._x * rate, v._y * rate, v._z * rate); }
	friend Vector3D operator *(const double& rate, const Vector3D& v){ return Vector3D(v._x * rate, v._y * rate, v._z * rate); }
	friend Vector3D operator *(const Vector3D& v1, const Vector3D& v2){ return Vector3D(v1._x * v2._x, v1._y * v2._y, v1._z * v2._z); }
	double _x, _y, _z;
};

double dot(Vector3D& v1, Vector3D& v2);

class Plane
{
public:
	Plane(Vector3D n, double d):_n(n), _d(d){}
	Plane(){}
	~Plane(){}

	Vector3D _n;
	double _d;
};

typedef Vector3D Color;

#endif