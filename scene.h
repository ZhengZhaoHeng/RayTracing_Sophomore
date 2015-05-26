#ifndef __SCENE_H_
#define __SCENE_H_

#include "common.h"
#include "Raytracer.h"
#include <string>
#include <vector>

#define HIT 1
#define MISS 0
#define INPRIM -1;


class Ray;
//Material Definition
class Material
{
public:
	Material();
	void set_color(Color& color){ _color = color; }
	Color get_color(){ return _color; }
	void set_ref(double refl){ _refl = refl; }
	double get_refl(){ return _refl; }
	void set_diff(double diff){ _diff = diff; }
	double get_diff(){ return _diff; }
	double get_spec(){ return 1.0 - _diff; }

private:
	Color _color;
	double _refl;
	double _diff;

};

//Primitive Definition
class Primitive
{
public:
	enum
	{
		SPHERE = 1,
		PLANE
	};
	Primitive():_light(false){}
	void set_material(Material material){ _material = material; }
	Material& get_material(){ return _material; }
	bool isLight(){ return _light; }
	void set_name(const char* name){ _name = name; }
	std::string get_name(){ return _name; }
	virtual void set_light(bool light){ _light = light; }
	virtual int get_type() = 0;
	virtual int intersect(Ray& ray, double& dist) = 0;
	virtual Vector3D get_normal(Vector3D& pos) = 0;
	virtual Color get_color(){ return _material.get_color(); }

protected:
	Material _material;
	std::string _name;
	bool _light;
};

class Sphere : public Primitive
{
public:
	Sphere(Vector3D& center, double radius):
		_center(center), _radius(radius),
		_sq_radius(sqr(radius)), _r_radius(1.0 / radius){}
	~Sphere(){}
	int get_type(){ return SPHERE; }
	Vector3D& get_center(){ return _center; }
	Vector3D get_normal(Vector3D& pos){ return (pos - _center) * _radius; }
	double get_sq_radius(){ return _sq_radius; }
	int intersect(Ray& ray, double& dist);

private:
	Vector3D _center;
	double _radius, _sq_radius, _r_radius;
};

class PlaneForPrim : public Primitive
{
public:
	PlaneForPrim(Plane& plane):_plane(plane){}
	~PlaneForPrim(){}
	int get_type(){ return PLANE; }
	Vector3D& get_normal(){ return _plane._n; }
	double get_d(){ return _plane._d; }
	int intersect(Ray& ray, double& dist);
	Vector3D get_normal(Vector3D& pos){ return _plane._n; }

private:
	Plane _plane;
};

class Scene
{
public:
	Scene(){}
	~Scene();
	void initScene();
	int get_num_prim(){ return _primitives.size(); }
	Primitive* get_primitive(int idx){ return _primitives[idx]; }
private:
	std::vector<Primitive*> _primitives;
};


#endif