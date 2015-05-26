#include "scene.h"

Material::Material() :
	_color( Color( 0.2, 0.2, 0.2 ) ),
	_refl( 0 ), _diff( 0.2 )
{
}

int Sphere::intersect(Ray& ray, double& dist)
{
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

int PlaneForPrim::intersect(Ray& ray, double& dist)
{
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

void Scene::initScene()
{
	// ground plane
	_primitives.push_back(new PlaneForPrim(Plane(Vector3D(0, 1, 0), 4.4)));
	_primitives.back()->set_name( "plane" );
	_primitives.back()->get_material().set_ref(0);
	_primitives.back()->get_material().set_diff(1.0);
	_primitives.back()->get_material().set_color(Color( 0.4, 0.3, 0.3 ));
	// big sphere
	_primitives.push_back(new Sphere( Vector3D( 1, -0.8, 3 ), 2.5 ));
	_primitives.back()->set_name("big sphere");
	_primitives.back()->get_material().set_ref(0.6);
	_primitives.back()->get_material().set_color(Color(0.7, 0.7, 0.7));
	// small sphere
	_primitives.push_back(new Sphere( Vector3D( -5.5, -0.5, 7 ), 2));
	_primitives.back()->set_name("small shphere");
	_primitives.back()->get_material().set_ref(1.0);
	_primitives.back()->get_material().set_diff(0.1);
	_primitives.back()->get_material().set_color(Color(0.7, 0.7, 1.0));
	// light source 1
	_primitives.push_back(new Sphere(Vector3D(0, 5, 5), 0.1));
	_primitives.back()->set_light(true);
	_primitives.back()->get_material().set_color(Color(0.4, 0.4, 0.4));
	// light source 2
	_primitives.push_back(new Sphere(Vector3D(2, 5, 1), 0.1));
	_primitives.back()->set_name("light");
	_primitives.back()->set_light(true);
	_primitives.back()->get_material().set_color(Color(0.6, 0.6, 0.8));
}

Scene::~Scene()
{
	for (int i = 0; i < _primitives.size(); i++)
	{
		delete _primitives[i];
	}
}
