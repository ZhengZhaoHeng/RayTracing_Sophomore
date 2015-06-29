#include "scene.h"
#include <cstdlib>
#include <ctime>
#include <cmath>

Material::Material() :
	_color( Color( 0.2, 0.2, 0.2 ) ),
	_refl( 0 ), _diff( 0.2 ), _refr(0), _refr_index(1.5), _spec(0.8), _diff_refl(0), _textured(false),
	_uscale(10), _vscale(10)
{
}

Material::~Material()
{
	cvReleaseImage(&_texture);
}

int Sphere::intersect(Ray& ray, double& dist)
{
	_ray_id = ray.get_id();
	Vector3D v = _center - ray.get_origin();
	double b = dot(v, ray.get_direction());
	double delta = _sq_radius + sqr(b) - dot(v, v);
	if (delta > 0)
	{
		delta = sqrt(delta);
		double i1 = b - delta;
		double i2 = b + delta;
		if (i2 > 0)
		{
			if (i1 < 0)
			{
				if (i2 < dist)
				{
					dist = i2;
					return INPRIM;
				}
			}
			else
			{
				if (i1 < dist)
				{
					dist = i1;
					return HIT;
				}
			}
		}

	}
	return MISS;
}

Color Sphere::get_color(Vector3D& pos)
{
	if (!_material.get_textured())
	{
		return _material.get_color();
	}
	else
	{
		Vector3D r = pos - _center;
		r.normalize();
		double phi = acos(r._z);
		double theta;
		if (abs(abs(r._z) - 1) < EPS) theta = 0;
		else
			if (r._x > 0)
			{
				if (r._y  > 0) theta = asin(r._y / sqrt(1 - sqr(r._z)));
				else theta = asin(r._y / sqrt(1 - sqr(r._z))) + 2 * PI;
			}
			else
			{
				if (r._y > 0) theta = acos(r._x / sqrt(1 - sqr(r._z)));
				else theta = PI - asin(r._y / sqrt(1 - sqr(r._z)));
			}
		return _material.get_texture(theta / (2 * PI), phi / PI);
	}
}

bool Sphere::intersectBox(AABB& box)
{
	double dmin = 0;
	Vector3D b_down = box.get_pos();
	Vector3D b_up = box.get_pos() + box.get_size();
	if (_center._x < b_down._x)
	{
		dmin += sqr(_center._x - b_down._x);
	}
	else if (_center._x > b_up._x)
	{
		dmin += sqr(_center._x - b_up._x);
	}
	if (_center._y < b_down._y)
	{
		dmin += sqr(_center._y - b_down._y);
	}
	else if (_center._y  > b_up._y)
	{
		dmin += sqr(_center._y - b_up._y);
	}
	if (_center._z < b_down._z)
	{
		dmin += sqr(_center._z - b_down._z);
	}
	else if (_center._z > b_up._z)
	{
		dmin += sqr(_center._z - b_up._z);
	}
	if (dmin - _sq_radius < EPS) return true;
	else return false;
}

AABB Sphere::get_box()
{
	Vector3D size(_radius, _radius, _radius);
	return AABB(_center - size, 2 * size);
}

int PlaneForPrim::intersect(Ray& ray, double& dist)
{
	_ray_id = ray.get_id();
	double proj  = dot(_plane._n, ray.get_direction());
	if (proj != 0)
	{
		double d = -(dot(_plane._n, ray.get_origin()) + _plane._d) / proj;
		if (d > 0 && d < dist)
		{
			dist = d;
			return HIT;
		}
	}
	return MISS;
}

bool PlaneForPrim::intersectBox(AABB& box)
{
	Vector3D v[2];
	v[0] = box.get_pos();
	v[1] = box.get_pos() + box.get_size();
	int side_1 = 0;
	int side_2 = 0;
	for (int i = 0; i < 8; i++)
	{
		Vector3D vertex(v[i & 1]._x , v[(i >> 1) & 1]._y, v[(i >> 2) & 1]._z);
		if ((dot(vertex, _plane._n) + _plane._d) < 0) side_1++;
		else side_2++;
	}
	if (side_1 == 0 && side_2 == 0) return false;
	return true;
}

Color PlaneForPrim::get_color(Vector3D& pos)
{
	if (!_material.get_textured()) return _material.get_color();
	else
	{
		Vector3D pnt = pos;
		pnt.normalize();
		double u = (dot(_vx, pnt) + 1) / 2;
		double v = (dot(_vy, pnt) + 1) / 2;
		return _material.get_texture(u, v);
	}
}


int Box::intersect(Ray& ray, double& dist)
{
	_ray_id = ray.get_id();
	double d[6];
	Vector3D ip[6];
	Vector3D dir = ray.get_direction();
	Vector3D src = ray.get_origin();
	int result = MISS;
	for (int i = 0; i < 6; i++)
	{
		d[i] = -1;
	}
	Vector3D b_down = _box.get_pos();
	Vector3D b_up = _box.get_pos() + _box.get_size();
	if (dir._x != 0)
	{
		double rc = 1.0 / dir._x;
		d[0] = (b_down._x - src._x) * rc;
		d[3] = (b_up._x - src._x) * rc;
	}
	if (dir._y != 0)
	{
		double rc = 1.0 / dir._y;
		d[1] = (b_down._y - src._y) * rc;
		d[4] = (b_up._y - src._y) * rc;
	}
	if (dir._z != 0)
	{
		double rc = 1.0 / dir._z;
		d[2] = (b_down._z - src._z) * rc;
		d[5] = (b_up._z - src._z) * rc;
	}
	for (int i = 0; i < 6; i++)
	{
		if (d[i] > EPS)
		{
			ip[i] = src + d[i] * dir;
			if ((ip[i]._x - b_down._x > -EPS) && (ip[i]._x - b_up._x < EPS) &&
				(ip[i]._y - b_down._y > -EPS) && (ip[i]._y - b_up._y < EPS) &&
				(ip[i]._z - b_down._z > -EPS) && (ip[i]._z - b_up._z < EPS))
			{
				if (d[i] < dist + EPS)
				{
					dist = d[i];
					result = HIT;
				}
			}
		}
	}
	return result;
}

Vector3D Box::get_normal(Vector3D& pnt)
{
	return Vector3D(0, 0, 0);
}

void Box::set_light(bool light)
{
	_light = light;
	if (!_grid)
	{
		_grid = new double[32];
		_grid[ 0] = 1, _grid[ 1] = 2;
		_grid[ 2] = 3, _grid[ 3] = 3;
		_grid[ 4] = 2, _grid[ 5] = 0;
		_grid[ 6] = 0, _grid[ 7] = 1;
		_grid[ 8] = 2, _grid[ 9] = 3;
		_grid[10] = 0, _grid[11] = 3;
		_grid[12] = 0, _grid[13] = 0;
		_grid[14] = 2, _grid[15] = 2;
		_grid[16] = 3, _grid[17] = 1;
		_grid[18] = 1, _grid[19] = 3;
		_grid[20] = 1, _grid[21] = 0;
		_grid[22] = 3, _grid[23] = 2;
		_grid[24] = 2, _grid[25] = 1;
		_grid[26] = 3, _grid[27] = 0;
		_grid[28] = 1, _grid[29] = 1;
		_grid[30] = 0, _grid[31] = 2;
		for ( int i = 0; i < 16; i++ )
		{
			_grid[i * 2] = _grid[i * 2] * _box.get_size()._x / 4 + _box.get_pos()._x;
			_grid[i * 2 + 1] = _grid[i * 2 + 1] * _box.get_size()._z / 4 + _box.get_pos()._z;
		}
	}
}

Triangle::Triangle(Vector3D p1, Vector3D p2, Vector3D p3):_p1(p1), _p2(p2), _p3(p3)
{
	Vector3D v1 = p2 - p1;
	Vector3D v2 = p3 - p1;
	_n = v1.cross(v2);
}

int Triangle::intersect(Ray& ray, double& dist)
{

}

bool intersectBox(AABB& box)
{
}



void Scene::initScene()
{
	srand((int)time(NULL));
	_primitives = new Primitive*[1000];
	// ground plane
	_primitives[0]= new PlaneForPrim(Plane(Vector3D(0, 1, 0), 4.4));
	_primitives[0]->set_name( "plane" );
	_primitives[0]->get_material().set_refl(0);
	_primitives[0]->get_material().set_refr(0);
	_primitives[0]->get_material().set_diff(1.0);
	_primitives[0]->get_material().set_color(Color( 0.4, 0.3, 0.3 ));
	// big sphere
	_primitives[1] = new Sphere( Vector3D( 2, 0.8, 3 ), 2.5 );
	_primitives[1]->set_name("big sphere");
	_primitives[1]->get_material().set_refl(0.2);
	_primitives[1]->get_material().set_refr(0.8);
	_primitives[1]->get_material().set_refr_index(1.3);
	_primitives[1]->get_material().set_color(Color(0.7, 0.7, 1));
	_primitives[1]->get_material().set_diff_refl(0.0);
	_primitives[1]->get_material().set_texture("texture1.jpg");
	// small sphere
	_primitives[2] = new Sphere( Vector3D( -5.5, -0.5, 7 ), 2);
	_primitives[2]->set_name("small shphere");
	_primitives[2]->get_material().set_refl(0.5);
	_primitives[2]->get_material().set_diff(0.2);
	_primitives[2]->get_material().set_spec(0.8);
	_primitives[2]->get_material().set_refr(0.2);
	_primitives[2]->get_material().set_refr_index(1.3);
	_primitives[2]->get_material().set_color(Color(1, 1, 1.0));
	_primitives[2]->get_material().set_diff_refl(0.0);
	_primitives[2]->get_material().set_texture("texture1.jpg");
	// light source 1
	_primitives[3] = new Sphere(Vector3D(0, 5, 0), 0.1);
	_primitives[3]->set_light(true);
	_primitives[3]->get_material().set_color(Color(0.6,0.6,0.8));
	// light source 2
	_primitives[4] = new Sphere(Vector3D(-3, 5, 1), 0.1);
	_primitives[4]->set_name("light");
	_primitives[4]->set_light(true);
	_primitives[4]->get_material().set_color(Color(1, 1, 1));
	// extra sphere
	_primitives[5] = new Sphere(Vector3D(-1.5, -3.8, 1), 1.5);
	_primitives[5]->set_name("extra shpere");
	_primitives[5]->get_material().set_refl(0);
	_primitives[5]->get_material().set_refr(0.8);
	_primitives[5]->get_material().set_color(Color(1, 0.4, 0.4));
	// back plane
	_primitives[6] = new PlaneForPrim(Plane(Vector3D(0.4, 0, -1), 12));
	_primitives[6]->set_name("back plane");
	_primitives[6]->get_material().set_refl(0);
	_primitives[6]->get_material().set_refr(0);
	_primitives[6]->get_material().set_spec(0);
	_primitives[6]->get_material().set_diff(0.6);
	_primitives[6]->get_material().set_color(Color(0.5, 0.3, 0.5));
	_primitives[6]->get_material().set_texture("texture1.jpg");
	// ceiling plane
	_primitives[7] = new PlaneForPrim(Plane(Vector3D(0, -1, 0), 7.4));
	_primitives[7]->set_name("ceiling plane");
	_primitives[7]->get_material().set_refl(0);
	_primitives[7]->get_material().set_refr(0);
	_primitives[7]->get_material().set_spec(0);
	_primitives[7]->get_material().set_diff(0.5);
	_primitives[7]->get_material().set_color(Color(0.4, 0.7, 0.7));
	_n_primitives = 8;
	//grid
	_primitives[8] = new Box( AABB( Vector3D( -1, 5, 4 ), Vector3D( 2, 0.1f, 2 ) ) );
	_primitives[8]->set_light( true );
	_primitives[8]->get_material().set_color( Color(0.6796,0.9297,0.9297) );
	_n_primitives++;
	/*
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 7; j++)
		{
			_primitives[_n_primitives] = new Sphere(Vector3D( -4.5 + i * 1.5, -4.3 + j * 1.5, 10 ), 0.3);
			_primitives[_n_primitives]->set_name("grid sphere");
			_primitives[_n_primitives]->get_material().set_refl(0);
			_primitives[_n_primitives]->get_material().set_refr(0);
			_primitives[_n_primitives]->get_material().set_spec(0.6);
			_primitives[_n_primitives]->get_material().set_diff(0.6);
			_primitives[_n_primitives]->get_material().set_color(Color(0.3, 1, 0.4));
			_n_primitives++;
		}
	}
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 8; j++)
		{
			_primitives[_n_primitives] = new Sphere(Vector3D( -4.5 + i * 1.5, -4.3, 10 - j * 1.5), 0.3);
			_primitives[_n_primitives]->set_name("grid sphere");
			_primitives[_n_primitives]->get_material().set_refl(0);
			_primitives[_n_primitives]->get_material().set_refr(0);
			_primitives[_n_primitives]->get_material().set_spec(0.6);
			_primitives[_n_primitives]->get_material().set_diff(0.6);
			_primitives[_n_primitives]->get_material().set_color(Color(0.3, 1, 0.4));
			_n_primitives++;
		}
	}
	for (int i = 0; i < 16; i++)
	{
		for (int j = 0; j < 8; j++)
		{
			_primitives[_n_primitives] = new Sphere(Vector3D( -8.5 + i * 1.5, 4.3, 10 - j), 0.3);
			_primitives[_n_primitives]->set_name("grid sphere");
			_primitives[_n_primitives]->get_material().set_refl(0);
			_primitives[_n_primitives]->get_material().set_refr(0);
			_primitives[_n_primitives]->get_material().set_spec(0.6);
			_primitives[_n_primitives]->get_material().set_diff(0.6);
			_primitives[_n_primitives]->get_material().set_color(Color(0.3, 1, 0.4));
			_n_primitives++;
		}
	}
	std::cout << _n_primitives << std::endl;*/
	buildGrid();
}

void Scene::buildGrid()
{
	_grid = new ObjList*[GRIDSIZE * GRIDSIZE * GRIDSIZE];
	memset(_grid, 0, GRIDSIZE * GRIDSIZE  * GRIDSIZE * 4);
	Vector3D down = Vector3D(-14, -5, -6); 
	Vector3D up = Vector3D(14, 8, 30);
	double dx = (up._x - down._x) / GRIDSIZE;
	double dx_reci = 1.0 / dx;
	double dy = (up._y - down._y) / GRIDSIZE;
	double dy_reci = 1.0 / dy;
	double dz = (up._z - down._z) / GRIDSIZE;
	double dz_reci = 1.0 / dz;
	_extends = AABB(down, up - down);
	_lights = new Primitive*[MAXLIGHTS];
	_n_lights = 0;
	for (int i = 0; i < _n_primitives; i++)
	{
		if (_primitives[i]->isLight())
		{
			_lights[_n_lights] = _primitives[i];
			_n_lights++;
		}
		AABB bound = _primitives[i]->get_box();
		Vector3D b_down = bound.get_pos();
		Vector3D b_up = bound.get_pos() + bound.get_size();
		int x1 = int((b_down._x - down._x) * dx_reci);
		int x2 = int((b_up._x - down._x) * dx_reci) + 1;
		x1 = std::max(x1, 0);
		x2 = std::min(x2, GRIDSIZE - 1);
		int y1 = int((b_down._y - down._y) * dy_reci);
		int y2 = int((b_up._y - down._y) * dy_reci) + 1;
		y1 = std::max(y1, 0);
		y2 = std::min(y2, GRIDSIZE - 1);
		int z1 = int((b_down._z - down._z) * dz_reci);
		int z2 = int((b_up._z - down._z) * dz_reci) + 1;
		z1 = std::max(z1, 0);
		z2 = std::min(z2, GRIDSIZE - 1);
		for (int x = x1; x < x2; x++)
		{
			for (int y = y1; y < y2; y++)
			{
				for (int z = z1; z < z2; z++)
				{
					int index = x + y * GRIDSIZE + z * GRIDSIZE * GRIDSIZE;
					Vector3D pos(down._x + x * dx, down._y + y * dy, down._z + z * dz);
					AABB cell(pos, Vector3D(dx, dy, dz));
					if (_primitives[i]->intersectBox(cell))
					{
						ObjList* l = new ObjList();
						l->set_primitive(_primitives[i]);
						l->set_next(_grid[index]);
						_grid[index] = l;
					}
				}
			}
		}
	}
}

Scene::~Scene()
{
	for (int i = 0; i < _n_primitives; i++)
	{
		delete _primitives[i];
	}
	delete _grid;
}
