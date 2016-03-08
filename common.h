#ifndef __COMMON_H_
#define __COMMON_H_

#include <cmath>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\opencv.hpp>
#include <iostream>
#include <thread>

#define GRIDSIZE 16
#define GRIDSHIFT 4
#define EPS 1e-4
#define TRACEDEPTH 6
#define KDTREE 1
#define DOF 0
#define SOFT 0
#define PI 3.1415926535

double sqr(double);

class Vector3D
{
public:
	Vector3D():_x(0), _y(0), _z(0){}
	Vector3D(double x, double y, double z):_x(x), _y(y), _z(z){}
	~Vector3D(){}
	void normalize();
	double length();
	Vector3D cross(Vector3D b) { return Vector3D( _y * b._z - _z * b._y, _z * b._x - _x * b._z, _x * b._y - _y * b._x ); }
	friend Vector3D operator +(const Vector3D& v1, const Vector3D& v2){ return Vector3D(v1._x + v2._x, v1._y + v2._y, v1._z + v2._z); }
	friend Vector3D operator -(const Vector3D& v1, const Vector3D& v2){ return Vector3D(v1._x - v2._x, v1._y - v2._y, v1._z - v2._z); }
	friend Vector3D operator /(const Vector3D& v, const double& rate){ return Vector3D(v._x / rate, v._y / rate, v._z / rate); }
	friend Vector3D operator *(const Vector3D& v, const double& rate){ return Vector3D(v._x * rate, v._y * rate, v._z * rate); }
	friend Vector3D operator *(const double& rate, const Vector3D& v){ return Vector3D(v._x * rate, v._y * rate, v._z * rate); }
	friend Vector3D operator *(const Vector3D& v1, const Vector3D& v2){ return Vector3D(v1._x * v2._x, v1._y * v2._y, v1._z * v2._z); }
	friend bool operator ==(const Vector3D& v1, const Vector3D& v2){ return v1._x == v2._x && v1._y == v2._y && v1._z == v2._z;}
	double _x, _y, _z;
};

double dot(Vector3D& v1, Vector3D& v2);
double det(Vector3D& v1, Vector3D& v2, Vector3D& v3);
void boxPro(Vector3D& p1, Vector3D& p2, Vector3D& n, double& mi, double& ma);
void triPro(Vector3D& p1, Vector3D& p2, Vector3D& p3, Vector3D& n, double& mi, double &ma);
bool check(double amax, double amin, double bmax, double bmin);

class Plane
{
public:
	Plane(Vector3D n, double d):_n(n), _d(d){}
	Plane(){}
	~Plane(){}

	Vector3D _n;
	double _d;
};

class AABB
{
public:
	AABB():_pos(Vector3D(0, 0, 0)), _size(Vector3D(0, 0, 0)) {}
	AABB( Vector3D& pos, Vector3D& size): _pos(pos), _size(size) {}
	Vector3D& get_pos() { return _pos; }
	Vector3D& get_size() { return _size; }
	bool intersect(AABB& b2)
	{
		Vector3D b1_down = _pos;
		Vector3D b1_up = _pos + _size;
		Vector3D b2_down = b2.get_pos();
		Vector3D b2_up = b2.get_pos()  + b2.get_size();
		if ((b1_up._x > b2_down._x) && (b1_down._x < b2_up._x) &&
			(b1_up._y > b2_down._y) && (b1_down._y < b2_up._y) &&
			(b1_up._z > b2_down._z) && (b1_down._z < b2_up._z))
		{
			return true;
		}
		return false;
	}
	bool contains(Vector3D pos)
	{
		Vector3D b_down = _pos;
		Vector3D b_up = _pos + _size;
		if ((b_down._x - pos._x < -EPS) && (b_up._x - pos._x > EPS) &&
			(b_down._y - pos._y < -EPS) && (b_up._y - pos._y > EPS) &&
			(b_down._z - pos._z < -EPS) && (b_up._z - pos._z > EPS))
		{
			return true;
		}
		return false;
	}
	Vector3D _size;
	Vector3D _pos;
};

class Matrix
{
public:
	enum 
	{ 
		TX=3, 
		TY=7, 
		TZ=11, 
		D0=0, D1=5, D2=10, D3=15, 
		SX=D0, SY=D1, SZ=D2, 
		W=D3 
	};
	Matrix() { identity(); }
	void identity()
	{
		_cell[1] = _cell[2] = _cell[TX] = _cell[4] = _cell[6] = _cell[TY] =
		_cell[8] = _cell[9] = _cell[TZ] = _cell[12] = _cell[13] = _cell[14] = 0;
		_cell[D0] = _cell[D1] = _cell[D2] = _cell[W] = 1;
	}
	void rotate( Vector3D pos, double rx, double ry, double rz )
	{
		Matrix t;
		t.rotateX( rz );
		rotateY( ry );
		concatenate( t );
		t.rotateZ( rx );
		concatenate( t );
		translate( pos );
	}
	void rotateX( double rx )
	{
		double sx = sin( rx * PI / 180 );
		double cx = cos( rx * PI / 180 );
		identity();
		_cell[5] = cx, _cell[6] = sx, _cell[9] = -sx, _cell[10] = cx;
	}
	void rotateY( double ry )
	{
		double sy = sin( ry * PI / 180 );
		double cy = cos( ry * PI / 180 );
		identity ();
		_cell[0] = cy, _cell[2] = -sy, _cell[8] = sy, _cell[10] = cy;
	}
	void rotateZ( double rz)
	{
		double sz = (double)sin( rz * PI / 180 );
		double cz = (double)cos( rz * PI / 180 );
		identity ();
		_cell[0] = cz, _cell[1] = sz, _cell[4] = -sz, _cell[5] = cz;
	}
	void translate( Vector3D pos ) 
	{ 
		_cell[TX] += pos._x; 
		_cell[TY] += pos._y; 
		_cell[TZ] += pos._z; 
	}
	void concatenate( Matrix& m2 )
	{
		Matrix res;
		for ( int c = 0; c < 4; c++ ) 
			for ( int r = 0; r < 4; r++ )
			res._cell[r * 4 + c] = _cell[r * 4] * m2._cell[c] +
				  				  _cell[r * 4 + 1] * m2._cell[c + 4] +
								  _cell[r * 4 + 2] * m2._cell[c + 8] +
								  _cell[r * 4 + 3] * m2._cell[c + 12];
		for (int c = 0; c < 16; c++ ) _cell[c] = res._cell[c];
	}
	Vector3D transform( Vector3D& v )
	{
		double x  = _cell[0] * v._x + _cell[1] * v._y + _cell[2] * v._z + _cell[3];
		double y  = _cell[4] * v._x + _cell[5] * v._y + _cell[6] * v._z + _cell[7];
		double z  = _cell[8] * v._x + _cell[9] * v._y + _cell[10] * v._z + _cell[11];
		return Vector3D( x, y, z );
	}
	void invert()
	{
		Matrix t;
		double tx = -_cell[3], ty = -_cell[7], tz = -_cell[11];
		for ( int h = 0; h < 3; h++ ) for ( int v = 0; v < 3; v++ ) t._cell[h + v * 4] = _cell[v + h * 4];
		for ( int i = 0; i < 11; i++ ) _cell[i] = t._cell[i];
		_cell[3] = tx * _cell[0] + ty * _cell[1] + tz * _cell[2];
		_cell[7] = tx * _cell[4] + ty * _cell[5] + tz * _cell[6];
		_cell[11] = tx * _cell[8] + ty * _cell[9] + tz * _cell[10];
	}
	double _cell[16];
};

typedef Vector3D Color;

#endif